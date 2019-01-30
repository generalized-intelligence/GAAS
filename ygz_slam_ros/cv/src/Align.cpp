#include "ygz/Settings.h"
#include "ygz/Align.h"

namespace ygz {

    // SSE 就交给你们了
    bool Align2D(
            const cv::Mat &cur_img,
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Vector2f &cur_px_estimate) {

        bool converged = false;

        // compute derivative of template and prepare inverse compositional
        float __attribute__ (( __aligned__ ( 16 ))) ref_patch_dx[align_patch_area];
        float __attribute__ (( __aligned__ ( 16 ))) ref_patch_dy[align_patch_area];
        Matrix3f H;
        H.setZero();

        // compute gradient and hessian
        const int ref_step = align_patch_size + 2;
        float *it_dx = ref_patch_dx;
        float *it_dy = ref_patch_dy;
        for (int y = 0; y < align_patch_size; ++y) {
            uint8_t *it = ref_patch_with_border + (y + 1) * ref_step + 1;
            for (int x = 0; x < align_patch_size; ++x, ++it, ++it_dx, ++it_dy) {
                Vector3f J;
                J[0] = 0.5 * (it[1] - it[-1]);
                J[1] = 0.5 * (it[ref_step] - it[-ref_step]);
                J[2] = 1;
                *it_dx = J[0];
                *it_dy = J[1];
                H += J * J.transpose();
            }
        }
        Matrix3f Hinv = H.inverse();

        // H is singular, maybe caused by over explosure or completely dark parts
        if (isnan(Hinv(0, 0))) {
            cur_px_estimate << -1, -1;
            return false;
        }

        float mean_diff = 0;

        // Compute pixel location in new image:
        float u = cur_px_estimate.x();
        float v = cur_px_estimate.y();

        // termination condition
        const float min_update_squared = 0.03 * 0.03;
        const int cur_step = cur_img.step.p[0];
        Vector3f update;
        update.setZero();
        float chi2 = 0;
        for (int iter = 0; iter < n_iter; ++iter) {
            chi2 = 0;
            int u_r = floor(u);
            int v_r = floor(v);
            if (u_r < align_halfpatch_size || v_r < align_halfpatch_size ||
                u_r >= cur_img.cols - align_halfpatch_size ||
                v_r >= cur_img.rows - align_halfpatch_size)
                break;

            // compute interpolation weights
            float subpix_x = u - u_r;
            float subpix_y = v - v_r;
            float wTL = (1.0 - subpix_x) * (1.0 - subpix_y);
            float wTR = subpix_x * (1.0 - subpix_y);
            float wBL = (1.0 - subpix_x) * subpix_y;
            float wBR = subpix_x * subpix_y;

            // loop through search_patch, interpolate
            uint8_t *it_ref = ref_patch;
            float *it_ref_dx = ref_patch_dx;
            float *it_ref_dy = ref_patch_dy;
            Vector3f Jres;
            Jres.setZero();
            for (int y = 0; y < align_patch_size; ++y) {
                uint8_t *it = (uint8_t *) cur_img.data + (v_r + y - align_halfpatch_size) * cur_step + u_r -
                              align_halfpatch_size;
                for (int x = 0; x < align_patch_size; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy) {
                    float search_pixel = wTL * it[0] + wTR * it[1] + wBL * it[cur_step] + wBR * it[cur_step + 1];
                    float res = search_pixel - *it_ref + mean_diff;
                    Jres[0] -= res * (*it_ref_dx);
                    Jres[1] -= res * (*it_ref_dy);
                    Jres[2] -= res;
                    chi2 += res * res;
                }
            }
            update = Hinv * Jres;
            u += update[0];
            v += update[1];

            mean_diff += update[2];
            if (update[0] * update[0] + update[1] * update[1] < min_update_squared) {
                converged = true;
                break;
            }
        }

        if (isnan(u) || isnan(v)) {
            cur_px_estimate << -1, -1;
            return false;
        }
        cur_px_estimate << u, v;
        return converged;
    }

    bool Align1D(
            const cv::Mat &cur_img,
            const Eigen::Vector2f &dir, // direction in which the patch is allowed to move
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Vector2f &cur_px_estimate,
            double &h_inv) {

        bool converged = false;

        // compute derivative of template and prepare inverse compositional
        float __attribute__ (( __aligned__ ( 16 ))) ref_patch_dv[align_patch_area];
        Matrix2f H;
        H.setZero();

        // compute gradient and hessian
        const int ref_step = align_patch_size + 2;
        float *it_dv = ref_patch_dv;
        for (int y = 0; y < align_patch_size; ++y) {
            uint8_t *it = ref_patch_with_border + (y + 1) * ref_step + 1;
            for (int x = 0; x < align_patch_size; ++x, ++it, ++it_dv) {
                Vector2f J;
                J[0] = 0.5 * (dir[0] * (it[1] - it[-1]) + dir[1] * (it[ref_step] - it[-ref_step]));
                J[1] = 1;
                *it_dv = J[0];
                H += J * J.transpose();
            }
        }
        h_inv = 1.0 / H(0, 0) * align_patch_size * align_patch_size;
        Matrix2f Hinv = H.inverse();
        float mean_diff = 0;

        // Compute pixel location in new image:
        float u = cur_px_estimate.x();
        float v = cur_px_estimate.y();

        // termination condition
        const float min_update_squared = 0.03 * 0.03;
        const int cur_step = cur_img.step.p[0];
        float chi2 = 0;
        Vector2f update;
        update.setZero();
        for (int iter = 0; iter < n_iter; ++iter) {
            int u_r = floor(u);
            int v_r = floor(v);
            if (u_r < align_halfpatch_size || v_r < align_halfpatch_size ||
                u_r >= cur_img.cols - align_halfpatch_size ||
                v_r >= cur_img.rows - align_halfpatch_size)
                break;

            if (isnan(u) ||
                isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
                return false;

            // compute interpolation weights
            float subpix_x = u - u_r;
            float subpix_y = v - v_r;
            float wTL = (1.0 - subpix_x) * (1.0 - subpix_y);
            float wTR = subpix_x * (1.0 - subpix_y);
            float wBL = (1.0 - subpix_x) * subpix_y;
            float wBR = subpix_x * subpix_y;

            // loop through search_patch, interpolate
            uint8_t *it_ref = ref_patch;
            float *it_ref_dv = ref_patch_dv;
            float new_chi2 = 0.0;
            Vector2f Jres;
            Jres.setZero();
            for (int y = 0; y < align_patch_size; ++y) {
                uint8_t *it = (uint8_t *) cur_img.data + (v_r + y - align_halfpatch_size) * cur_step + u_r -
                              align_halfpatch_size;
                for (int x = 0; x < align_patch_size; ++x, ++it, ++it_ref, ++it_ref_dv) {
                    float search_pixel = wTL * it[0] + wTR * it[1] + wBL * it[cur_step] + wBR * it[cur_step + 1];
                    float res = search_pixel - *it_ref + mean_diff;
                    Jres[0] -= res * (*it_ref_dv);
                    Jres[1] -= res;
                    new_chi2 += res * res;
                }
            }

            if (iter > 0 && new_chi2 > chi2) {
                u -= update[0];
                v -= update[1];
                break;
            }

            chi2 = new_chi2;
            update = Hinv * Jres;
            u += update[0] * dir[0];
            v += update[0] * dir[1];
            mean_diff += update[1];


            if (update[0] * update[0] + update[1] * update[1] < min_update_squared) {
                converged = true;
                break;
            }
        }

        cur_px_estimate << u, v;
        return converged;
    }

}
