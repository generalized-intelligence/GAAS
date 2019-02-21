#include <vector>
#include <fast/fast.h>

// This is mechanically generated code.

namespace fast
{

void fast_corner_detect_10(const fast_byte* img, int img_width, int img_height, int img_stride,
                           short barrier, std::vector<fast_xy>& corners)
{
  int y, cb, c_b;
  const fast_byte  *line_max, *line_min;
  const fast_byte* cache_0;

  int pixel[16] = {
    0 + img_stride * 3,
    1 + img_stride * 3,
    2 + img_stride * 2,
    3 + img_stride * 1,
    3 + img_stride * 0,
    3 + img_stride * -1,
    2 + img_stride * -2,
    1 + img_stride * -3,
    0 + img_stride * -3,
    -1 + img_stride * -3,
    -2 + img_stride * -2,
    -3 + img_stride * -1,
    -3 + img_stride * 0,
    -3 + img_stride * 1,
    -2 + img_stride * 2,
    -1 + img_stride * 3,
  };

  for(y = 0 ; y < img_height ; y++)
  {
    cache_0 = img + y*img_stride ;
    line_min = cache_0 ;
    line_max = img + y*img_stride + img_width ;
    // cache_0 = &i[y][3];
    // line_min = cache_0 - 3;
    // line_max = &i[y][i.size().x - 3];


    for(; cache_0 < line_max;cache_0++)
    {
            cb = *cache_0 + barrier;
            c_b= *cache_0 - barrier;

            if(*(cache_0 + pixel[0]) > cb)
             if(*(cache_0 + pixel[8]) > cb)
              if(*(cache_0 + pixel[3]) > cb)
               if(*(cache_0 + pixel[5]) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + 3) > cb)
                   if(*(cache_0 + pixel[7]) > cb)
                    if(*(cache_0 + pixel[1]) > cb)
                     if(*(cache_0 + pixel[9]) > cb)
                      goto success;
                     else
                      if(*(cache_0 + pixel[15]) > cb)
                       goto success;
                      else
                       continue;
                    else if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[9]) > cb)
                      if(*(cache_0 + pixel[10]) > cb)
                       if(*(cache_0 + pixel[11]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + pixel[11]) > cb)
                      if(*(cache_0 + pixel[10]) > cb)
                       if(*(cache_0 + pixel[9]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else if(*(cache_0 + pixel[7]) < c_b)
                    if(*(cache_0 + pixel[1]) > cb)
                     if(*(cache_0 + pixel[13]) > cb)
                      if(*(cache_0 + pixel[14]) > cb)
                       if(*(cache_0 + pixel[15]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[14]) > cb)
                      if(*(cache_0 + pixel[15]) > cb)
                       if(*(cache_0 + pixel[1]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else if(*(cache_0 + 3) < c_b)
                   if(*(cache_0 + pixel[10]) > cb)
                    if(*(cache_0 + pixel[11]) > cb)
                     if(*(cache_0 + -3) > cb)
                      if(*(cache_0 + pixel[13]) > cb)
                       if(*(cache_0 + pixel[14]) > cb)
                        if(*(cache_0 + pixel[1]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          if(*(cache_0 + pixel[7]) > cb)
                           if(*(cache_0 + pixel[9]) > cb)
                            goto success;
                           else
                            continue;
                          else
                           continue;
                        else
                         if(*(cache_0 + pixel[7]) > cb)
                          if(*(cache_0 + pixel[9]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + -3) > cb)
                    if(*(cache_0 + pixel[14]) > cb)
                     if(*(cache_0 + pixel[10]) > cb)
                      if(*(cache_0 + pixel[11]) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[1]) > cb)
                         if(*(cache_0 + pixel[7]) > cb)
                          if(*(cache_0 + pixel[9]) > cb)
                           goto success;
                          else
                           if(*(cache_0 + pixel[15]) > cb)
                            goto success;
                           else
                            continue;
                         else
                          if(*(cache_0 + pixel[15]) > cb)
                           goto success;
                          else
                           continue;
                        else if(*(cache_0 + pixel[1]) < c_b)
                         if(*(cache_0 + pixel[7]) > cb)
                          if(*(cache_0 + pixel[9]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         if(*(cache_0 + pixel[9]) > cb)
                          if(*(cache_0 + pixel[7]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + -3) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    if(*(cache_0 + pixel[14]) > cb)
                     if(*(cache_0 + pixel[15]) > cb)
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + 3) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[10]) > cb)
                         if(*(cache_0 + pixel[11]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[9]) > cb)
                         if(*(cache_0 + pixel[10]) > cb)
                          if(*(cache_0 + pixel[11]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  if(*(cache_0 + -3) > cb)
                   if(*(cache_0 + pixel[14]) > cb)
                    if(*(cache_0 + pixel[15]) > cb)
                     if(*(cache_0 + pixel[13]) > cb)
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + 3) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[10]) > cb)
                         if(*(cache_0 + pixel[11]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[9]) > cb)
                         if(*(cache_0 + pixel[10]) > cb)
                          if(*(cache_0 + pixel[11]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[10]) > cb)
                         if(*(cache_0 + pixel[11]) > cb)
                          if(*(cache_0 + pixel[9]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else if(*(cache_0 + pixel[2]) < c_b)
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   if(*(cache_0 + pixel[10]) > cb)
                    if(*(cache_0 + pixel[11]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[6]) > cb)
                       if(*(cache_0 + 3) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[13]) > cb)
                         if(*(cache_0 + pixel[14]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + pixel[10]) > cb)
                   if(*(cache_0 + -3) > cb)
                    if(*(cache_0 + pixel[9]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[6]) > cb)
                       if(*(cache_0 + 3) > cb)
                        goto success;
                       else if(*(cache_0 + 3) < c_b)
                        if(*(cache_0 + pixel[13]) > cb)
                         if(*(cache_0 + pixel[14]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[13]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else if(*(cache_0 + pixel[6]) < c_b)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       if(*(cache_0 + pixel[14]) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                     else if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      if(*(cache_0 + pixel[14]) > cb)
                       if(*(cache_0 + pixel[1]) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else if(*(cache_0 + pixel[5]) < c_b)
                if(*(cache_0 + pixel[13]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + -3) > cb)
                   if(*(cache_0 + pixel[14]) > cb)
                    if(*(cache_0 + pixel[15]) > cb)
                     if(*(cache_0 + pixel[10]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[1]) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[1]) > cb)
                        if(*(cache_0 + pixel[2]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + pixel[2]) > cb)
                        if(*(cache_0 + 3) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[11]) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    if(*(cache_0 + pixel[10]) > cb)
                     if(*(cache_0 + pixel[13]) > cb)
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + pixel[2]) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[9]) > cb)
                         goto success;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[9]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      continue;
                    else if(*(cache_0 + pixel[10]) < c_b)
                     if(*(cache_0 + pixel[1]) > cb)
                      if(*(cache_0 + pixel[2]) > cb)
                       if(*(cache_0 + 3) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + 3) > cb)
                      if(*(cache_0 + pixel[2]) > cb)
                       if(*(cache_0 + pixel[1]) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else if(*(cache_0 + pixel[3]) < c_b)
               if(*(cache_0 + -3) > cb)
                if(*(cache_0 + pixel[10]) > cb)
                 if(*(cache_0 + pixel[13]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    if(*(cache_0 + pixel[14]) > cb)
                     if(*(cache_0 + pixel[15]) > cb)
                      if(*(cache_0 + pixel[7]) > cb)
                       goto success;
                      else
                       if(*(cache_0 + pixel[1]) > cb)
                        goto success;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + pixel[6]) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     if(*(cache_0 + 3) > cb)
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + pixel[6]) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + -3) > cb)
                if(*(cache_0 + pixel[10]) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[11]) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    if(*(cache_0 + pixel[9]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[15]) > cb)
                       goto success;
                      else
                       if(*(cache_0 + pixel[5]) > cb)
                        if(*(cache_0 + pixel[6]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + pixel[15]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + 3) > cb)
                   if(*(cache_0 + pixel[5]) > cb)
                    if(*(cache_0 + pixel[6]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[11]) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  if(*(cache_0 + 3) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    if(*(cache_0 + pixel[6]) > cb)
                     if(*(cache_0 + pixel[11]) > cb)
                      if(*(cache_0 + pixel[7]) > cb)
                       if(*(cache_0 + pixel[5]) > cb)
                        if(*(cache_0 + pixel[9]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 continue;
               else
                continue;
             else if(*(cache_0 + pixel[8]) < c_b)
              if(*(cache_0 + pixel[11]) > cb)
               if(*(cache_0 + pixel[2]) > cb)
                if(*(cache_0 + pixel[15]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    if(*(cache_0 + pixel[3]) > cb)
                     if(*(cache_0 + -3) > cb)
                      if(*(cache_0 + 3) > cb)
                       goto success;
                      else
                       if(*(cache_0 + pixel[10]) > cb)
                        goto success;
                       else
                        continue;
                     else
                      if(*(cache_0 + 3) > cb)
                       if(*(cache_0 + pixel[5]) > cb)
                        if(*(cache_0 + pixel[6]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     if(*(cache_0 + pixel[9]) > cb)
                      if(*(cache_0 + pixel[10]) > cb)
                       if(*(cache_0 + -3) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    if(*(cache_0 + pixel[3]) > cb)
                     if(*(cache_0 + 3) > cb)
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + pixel[6]) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[2]) < c_b)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[3]) < c_b)
                  if(*(cache_0 + 3) < c_b)
                   if(*(cache_0 + pixel[5]) < c_b)
                    if(*(cache_0 + pixel[6]) < c_b)
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[9]) < c_b)
                       if(*(cache_0 + pixel[10]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + pixel[11]) < c_b)
               if(*(cache_0 + pixel[6]) > cb)
                if(*(cache_0 + pixel[14]) > cb)
                 if(*(cache_0 + pixel[3]) > cb)
                  if(*(cache_0 + pixel[1]) > cb)
                   if(*(cache_0 + pixel[2]) > cb)
                    if(*(cache_0 + 3) > cb)
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + pixel[15]) > cb)
                       if(*(cache_0 + pixel[7]) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[6]) < c_b)
                if(*(cache_0 + pixel[10]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  if(*(cache_0 + pixel[2]) > cb)
                   if(*(cache_0 + pixel[3]) > cb)
                    if(*(cache_0 + 3) > cb)
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + -3) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[10]) < c_b)
                 if(*(cache_0 + pixel[5]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   if(*(cache_0 + pixel[1]) > cb)
                    if(*(cache_0 + pixel[2]) > cb)
                     if(*(cache_0 + pixel[3]) > cb)
                      if(*(cache_0 + 3) > cb)
                       if(*(cache_0 + -3) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         if(*(cache_0 + pixel[14]) > cb)
                          if(*(cache_0 + pixel[15]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else if(*(cache_0 + pixel[7]) < c_b)
                   if(*(cache_0 + pixel[14]) > cb)
                    if(*(cache_0 + -3) > cb)
                     if(*(cache_0 + pixel[1]) > cb)
                      if(*(cache_0 + pixel[2]) > cb)
                       if(*(cache_0 + pixel[3]) > cb)
                        if(*(cache_0 + 3) > cb)
                         if(*(cache_0 + pixel[13]) > cb)
                          if(*(cache_0 + pixel[15]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[9]) < c_b)
                     if(*(cache_0 + -3) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + -3) > cb)
                    if(*(cache_0 + pixel[1]) > cb)
                     if(*(cache_0 + pixel[2]) > cb)
                      if(*(cache_0 + pixel[3]) > cb)
                       if(*(cache_0 + 3) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         if(*(cache_0 + pixel[14]) > cb)
                          if(*(cache_0 + pixel[15]) > cb)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else if(*(cache_0 + pixel[5]) < c_b)
                  if(*(cache_0 + -3) > cb)
                   if(*(cache_0 + pixel[2]) < c_b)
                    if(*(cache_0 + pixel[3]) < c_b)
                     if(*(cache_0 + 3) < c_b)
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[9]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else if(*(cache_0 + -3) < c_b)
                   if(*(cache_0 + pixel[9]) < c_b)
                    if(*(cache_0 + 3) > cb)
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[14]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else if(*(cache_0 + 3) < c_b)
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       goto success;
                      else
                       if(*(cache_0 + pixel[3]) < c_b)
                        goto success;
                       else
                        continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + pixel[14]) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + pixel[2]) < c_b)
                    if(*(cache_0 + pixel[7]) < c_b)
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + pixel[9]) < c_b)
                       if(*(cache_0 + 3) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(*(cache_0 + pixel[15]) < c_b)
                   if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[7]) < c_b)
                     if(*(cache_0 + pixel[9]) < c_b)
                      if(*(cache_0 + -3) < c_b)
                       if(*(cache_0 + pixel[13]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[1]) > cb)
                   if(*(cache_0 + pixel[2]) > cb)
                    if(*(cache_0 + pixel[3]) > cb)
                     if(*(cache_0 + 3) > cb)
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + pixel[3]) > cb)
                  if(*(cache_0 + pixel[1]) > cb)
                   if(*(cache_0 + pixel[2]) > cb)
                    if(*(cache_0 + 3) > cb)
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + pixel[13]) > cb)
                       if(*(cache_0 + pixel[14]) > cb)
                        if(*(cache_0 + pixel[15]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + pixel[3]) > cb)
                if(*(cache_0 + pixel[5]) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[15]) > cb)
                   if(*(cache_0 + pixel[13]) > cb)
                    if(*(cache_0 + pixel[1]) > cb)
                     if(*(cache_0 + pixel[2]) > cb)
                      if(*(cache_0 + 3) > cb)
                       if(*(cache_0 + pixel[6]) > cb)
                        goto success;
                       else
                        if(*(cache_0 + -3) > cb)
                         goto success;
                        else
                         continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[13]) < c_b)
                    if(*(cache_0 + pixel[6]) > cb)
                     if(*(cache_0 + pixel[1]) > cb)
                      if(*(cache_0 + pixel[2]) > cb)
                       if(*(cache_0 + 3) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[7]) > cb)
                     if(*(cache_0 + pixel[1]) > cb)
                      if(*(cache_0 + pixel[2]) > cb)
                       if(*(cache_0 + 3) > cb)
                        if(*(cache_0 + pixel[6]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[3]) < c_b)
                if(*(cache_0 + pixel[1]) < c_b)
                 if(*(cache_0 + pixel[10]) < c_b)
                  if(*(cache_0 + pixel[2]) < c_b)
                   if(*(cache_0 + 3) < c_b)
                    if(*(cache_0 + pixel[5]) < c_b)
                     if(*(cache_0 + pixel[6]) < c_b)
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[9]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(*(cache_0 + pixel[3]) > cb)
               if(*(cache_0 + pixel[14]) > cb)
                if(*(cache_0 + -3) > cb)
                 if(*(cache_0 + pixel[2]) > cb)
                  if(*(cache_0 + 3) > cb)
                   if(*(cache_0 + pixel[15]) > cb)
                    if(*(cache_0 + pixel[1]) > cb)
                     if(*(cache_0 + pixel[13]) > cb)
                      if(*(cache_0 + pixel[11]) > cb)
                       goto success;
                      else
                       if(*(cache_0 + pixel[5]) > cb)
                        goto success;
                       else
                        continue;
                     else if(*(cache_0 + pixel[13]) < c_b)
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + pixel[6]) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      if(*(cache_0 + pixel[7]) > cb)
                       if(*(cache_0 + pixel[5]) > cb)
                        if(*(cache_0 + pixel[6]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else if(*(cache_0 + 3) < c_b)
                   if(*(cache_0 + pixel[1]) > cb)
                    if(*(cache_0 + pixel[10]) > cb)
                     if(*(cache_0 + pixel[11]) > cb)
                      if(*(cache_0 + pixel[13]) > cb)
                       if(*(cache_0 + pixel[15]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + pixel[10]) > cb)
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[11]) > cb)
                      if(*(cache_0 + pixel[15]) > cb)
                       if(*(cache_0 + pixel[1]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else if(*(cache_0 + -3) < c_b)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + pixel[1]) > cb)
                   if(*(cache_0 + pixel[2]) > cb)
                    if(*(cache_0 + 3) > cb)
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + pixel[15]) > cb)
                       if(*(cache_0 + pixel[7]) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + pixel[2]) > cb)
                   if(*(cache_0 + pixel[5]) > cb)
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[15]) > cb)
                      if(*(cache_0 + 3) > cb)
                       if(*(cache_0 + pixel[1]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[1]) > cb)
                      if(*(cache_0 + 3) > cb)
                       if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[15]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[15]) > cb)
                       if(*(cache_0 + 3) > cb)
                        if(*(cache_0 + pixel[1]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                continue;
              else if(*(cache_0 + pixel[3]) < c_b)
               if(*(cache_0 + pixel[2]) > cb)
                if(*(cache_0 + pixel[9]) > cb)
                 if(*(cache_0 + pixel[1]) > cb)
                  if(*(cache_0 + pixel[10]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    if(*(cache_0 + -3) > cb)
                     if(*(cache_0 + pixel[13]) > cb)
                      if(*(cache_0 + pixel[14]) > cb)
                       if(*(cache_0 + pixel[15]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else
               if(*(cache_0 + pixel[9]) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[14]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[15]) > cb)
                      if(*(cache_0 + pixel[10]) > cb)
                       if(*(cache_0 + pixel[1]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
            else if(*(cache_0 + pixel[0]) < c_b)
             if(*(cache_0 + pixel[8]) > cb)
              if(*(cache_0 + pixel[2]) > cb)
               if(*(cache_0 + pixel[10]) > cb)
                if(*(cache_0 + pixel[6]) > cb)
                 if(*(cache_0 + pixel[7]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   if(*(cache_0 + pixel[5]) > cb)
                    if(*(cache_0 + pixel[11]) > cb)
                     if(*(cache_0 + 3) > cb)
                      if(*(cache_0 + pixel[3]) > cb)
                       goto success;
                      else
                       if(*(cache_0 + -3) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + -3) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     if(*(cache_0 + pixel[1]) > cb)
                      if(*(cache_0 + pixel[3]) > cb)
                       if(*(cache_0 + 3) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else if(*(cache_0 + pixel[5]) < c_b)
                    if(*(cache_0 + pixel[11]) > cb)
                     if(*(cache_0 + -3) > cb)
                      if(*(cache_0 + pixel[13]) > cb)
                       if(*(cache_0 + pixel[14]) > cb)
                        if(*(cache_0 + pixel[15]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[11]) > cb)
                      if(*(cache_0 + -3) > cb)
                       if(*(cache_0 + pixel[14]) > cb)
                        if(*(cache_0 + pixel[15]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + pixel[2]) < c_b)
               if(*(cache_0 + pixel[13]) > cb)
                if(*(cache_0 + pixel[6]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   if(*(cache_0 + pixel[7]) > cb)
                    if(*(cache_0 + pixel[10]) > cb)
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + -3) > cb)
                       if(*(cache_0 + 3) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[14]) > cb)
                         goto success;
                        else
                         continue;
                      else
                       continue;
                     else
                      if(*(cache_0 + pixel[15]) > cb)
                       if(*(cache_0 + -3) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[7]) < c_b)
                  if(*(cache_0 + pixel[1]) < c_b)
                   if(*(cache_0 + pixel[3]) < c_b)
                    if(*(cache_0 + 3) < c_b)
                     if(*(cache_0 + pixel[5]) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[13]) < c_b)
                if(*(cache_0 + pixel[3]) > cb)
                 if(*(cache_0 + pixel[10]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   if(*(cache_0 + 3) > cb)
                    if(*(cache_0 + pixel[5]) > cb)
                     if(*(cache_0 + pixel[6]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[11]) > cb)
                        if(*(cache_0 + -3) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else if(*(cache_0 + pixel[10]) < c_b)
                  if(*(cache_0 + pixel[9]) < c_b)
                   if(*(cache_0 + pixel[1]) < c_b)
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + -3) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[15]) < c_b)
                  if(*(cache_0 + pixel[1]) < c_b)
                   if(*(cache_0 + pixel[5]) > cb)
                    if(*(cache_0 + pixel[10]) < c_b)
                     if(*(cache_0 + pixel[14]) < c_b)
                      if(*(cache_0 + pixel[11]) < c_b)
                       if(*(cache_0 + -3) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + 3) < c_b)
                      if(*(cache_0 + pixel[11]) < c_b)
                       if(*(cache_0 + -3) < c_b)
                        if(*(cache_0 + pixel[14]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else if(*(cache_0 + pixel[5]) < c_b)
                    if(*(cache_0 + 3) < c_b)
                     if(*(cache_0 + pixel[6]) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       goto success;
                      else
                       continue;
                     else
                      if(*(cache_0 + -3) < c_b)
                       if(*(cache_0 + pixel[14]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                    else
                     if(*(cache_0 + pixel[10]) < c_b)
                      if(*(cache_0 + pixel[11]) < c_b)
                       if(*(cache_0 + -3) < c_b)
                        if(*(cache_0 + pixel[14]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + pixel[10]) > cb)
                      if(*(cache_0 + 3) < c_b)
                       if(*(cache_0 + -3) < c_b)
                        if(*(cache_0 + pixel[14]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else if(*(cache_0 + pixel[10]) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + -3) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      if(*(cache_0 + 3) < c_b)
                       if(*(cache_0 + pixel[14]) < c_b)
                        if(*(cache_0 + -3) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else
                 if(*(cache_0 + pixel[9]) < c_b)
                  if(*(cache_0 + pixel[11]) < c_b)
                   if(*(cache_0 + pixel[1]) < c_b)
                    if(*(cache_0 + pixel[10]) < c_b)
                     if(*(cache_0 + -3) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[7]) > cb)
                 if(*(cache_0 + pixel[3]) > cb)
                  if(*(cache_0 + pixel[10]) > cb)
                   if(*(cache_0 + 3) > cb)
                    if(*(cache_0 + pixel[5]) > cb)
                     if(*(cache_0 + pixel[6]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[11]) > cb)
                        if(*(cache_0 + -3) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[7]) < c_b)
                 if(*(cache_0 + pixel[1]) < c_b)
                  if(*(cache_0 + pixel[3]) < c_b)
                   if(*(cache_0 + 3) < c_b)
                    if(*(cache_0 + pixel[5]) < c_b)
                     if(*(cache_0 + pixel[6]) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + -3) > cb)
                if(*(cache_0 + pixel[6]) > cb)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + pixel[9]) > cb)
                   if(*(cache_0 + pixel[10]) > cb)
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + 3) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[14]) > cb)
                         goto success;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[15]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + pixel[3]) > cb)
                      if(*(cache_0 + 3) > cb)
                       if(*(cache_0 + pixel[5]) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else if(*(cache_0 + pixel[8]) < c_b)
              if(*(cache_0 + 3) > cb)
               if(*(cache_0 + -3) < c_b)
                if(*(cache_0 + pixel[10]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + pixel[15]) < c_b)
                   if(*(cache_0 + pixel[13]) < c_b)
                    if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[11]) < c_b)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[2]) < c_b)
                        if(*(cache_0 + pixel[3]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else if(*(cache_0 + pixel[9]) < c_b)
                       goto success;
                      else
                       if(*(cache_0 + pixel[3]) < c_b)
                        if(*(cache_0 + pixel[2]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[9]) < c_b)
                       if(*(cache_0 + pixel[11]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + pixel[5]) < c_b)
                    if(*(cache_0 + pixel[6]) < c_b)
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[9]) < c_b)
                       if(*(cache_0 + pixel[11]) < c_b)
                        if(*(cache_0 + pixel[13]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
              else if(*(cache_0 + 3) < c_b)
               if(*(cache_0 + pixel[2]) > cb)
                if(*(cache_0 + pixel[10]) < c_b)
                 if(*(cache_0 + -3) < c_b)
                  if(*(cache_0 + pixel[11]) < c_b)
                   if(*(cache_0 + pixel[9]) < c_b)
                    if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[14]) < c_b)
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[15]) > cb)
                        if(*(cache_0 + pixel[5]) < c_b)
                         if(*(cache_0 + pixel[6]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        if(*(cache_0 + pixel[6]) < c_b)
                         if(*(cache_0 + pixel[5]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[1]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[6]) < c_b)
                        if(*(cache_0 + pixel[7]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[6]) < c_b)
                        if(*(cache_0 + pixel[7]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + pixel[2]) < c_b)
                if(*(cache_0 + pixel[6]) > cb)
                 if(*(cache_0 + pixel[13]) < c_b)
                  if(*(cache_0 + pixel[14]) < c_b)
                   if(*(cache_0 + pixel[15]) < c_b)
                    if(*(cache_0 + -3) < c_b)
                     if(*(cache_0 + pixel[1]) < c_b)
                      if(*(cache_0 + pixel[3]) < c_b)
                       if(*(cache_0 + pixel[11]) < c_b)
                        goto success;
                       else
                        if(*(cache_0 + pixel[5]) < c_b)
                         goto success;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[9]) < c_b)
                        if(*(cache_0 + pixel[10]) < c_b)
                         if(*(cache_0 + pixel[11]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[9]) < c_b)
                        if(*(cache_0 + pixel[10]) < c_b)
                         if(*(cache_0 + pixel[11]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[3]) > cb)
                  if(*(cache_0 + pixel[9]) < c_b)
                   if(*(cache_0 + pixel[10]) < c_b)
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + -3) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[5]) < c_b)
                         goto success;
                        else
                         if(*(cache_0 + pixel[14]) < c_b)
                          if(*(cache_0 + pixel[15]) < c_b)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                       else
                        if(*(cache_0 + pixel[1]) < c_b)
                         if(*(cache_0 + pixel[14]) < c_b)
                          if(*(cache_0 + pixel[15]) < c_b)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else if(*(cache_0 + pixel[3]) < c_b)
                  if(*(cache_0 + pixel[5]) > cb)
                   if(*(cache_0 + pixel[11]) < c_b)
                    if(*(cache_0 + -3) < c_b)
                     if(*(cache_0 + pixel[13]) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        if(*(cache_0 + pixel[1]) < c_b)
                         goto success;
                        else
                         if(*(cache_0 + pixel[7]) < c_b)
                          if(*(cache_0 + pixel[9]) < c_b)
                           if(*(cache_0 + pixel[10]) < c_b)
                            goto success;
                           else
                            continue;
                          else
                           continue;
                         else
                          continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else if(*(cache_0 + pixel[5]) < c_b)
                   if(*(cache_0 + pixel[7]) > cb)
                    if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[13]) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[7]) < c_b)
                    if(*(cache_0 + pixel[1]) > cb)
                     if(*(cache_0 + pixel[9]) < c_b)
                      if(*(cache_0 + pixel[10]) < c_b)
                       if(*(cache_0 + pixel[11]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[9]) < c_b)
                      goto success;
                     else
                      if(*(cache_0 + pixel[15]) < c_b)
                       goto success;
                      else
                       continue;
                    else
                     if(*(cache_0 + pixel[11]) < c_b)
                      if(*(cache_0 + pixel[10]) < c_b)
                       if(*(cache_0 + pixel[9]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[15]) < c_b)
                      if(*(cache_0 + pixel[14]) < c_b)
                       if(*(cache_0 + pixel[1]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   if(*(cache_0 + -3) < c_b)
                    if(*(cache_0 + pixel[14]) < c_b)
                     if(*(cache_0 + pixel[11]) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        if(*(cache_0 + pixel[1]) > cb)
                         if(*(cache_0 + pixel[7]) < c_b)
                          if(*(cache_0 + pixel[9]) < c_b)
                           if(*(cache_0 + pixel[10]) < c_b)
                            goto success;
                           else
                            continue;
                          else
                           continue;
                         else
                          continue;
                        else if(*(cache_0 + pixel[1]) < c_b)
                         goto success;
                        else
                         if(*(cache_0 + pixel[9]) < c_b)
                          if(*(cache_0 + pixel[7]) < c_b)
                           if(*(cache_0 + pixel[10]) < c_b)
                            goto success;
                           else
                            continue;
                          else
                           continue;
                         else
                          continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  if(*(cache_0 + pixel[11]) < c_b)
                   if(*(cache_0 + pixel[13]) < c_b)
                    if(*(cache_0 + pixel[10]) < c_b)
                     if(*(cache_0 + pixel[9]) < c_b)
                      if(*(cache_0 + -3) < c_b)
                       if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[1]) < c_b)
                         if(*(cache_0 + pixel[14]) < c_b)
                          if(*(cache_0 + pixel[15]) < c_b)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                       else if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[5]) < c_b)
                         goto success;
                        else
                         if(*(cache_0 + pixel[14]) < c_b)
                          if(*(cache_0 + pixel[15]) < c_b)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                       else
                        if(*(cache_0 + pixel[15]) < c_b)
                         if(*(cache_0 + pixel[1]) < c_b)
                          if(*(cache_0 + pixel[14]) < c_b)
                           goto success;
                          else
                           continue;
                         else
                          continue;
                        else
                         continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                else
                 if(*(cache_0 + -3) < c_b)
                  if(*(cache_0 + pixel[14]) < c_b)
                   if(*(cache_0 + pixel[15]) < c_b)
                    if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[11]) > cb)
                      if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[3]) < c_b)
                        if(*(cache_0 + pixel[5]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else if(*(cache_0 + pixel[11]) < c_b)
                      if(*(cache_0 + pixel[1]) > cb)
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[9]) < c_b)
                         if(*(cache_0 + pixel[10]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[3]) > cb)
                        if(*(cache_0 + pixel[9]) < c_b)
                         if(*(cache_0 + pixel[10]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else if(*(cache_0 + pixel[3]) < c_b)
                        goto success;
                       else
                        if(*(cache_0 + pixel[10]) < c_b)
                         if(*(cache_0 + pixel[9]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[10]) < c_b)
                         if(*(cache_0 + pixel[9]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[3]) < c_b)
                        if(*(cache_0 + pixel[1]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[11]) < c_b)
                 if(*(cache_0 + pixel[10]) < c_b)
                  if(*(cache_0 + -3) < c_b)
                   if(*(cache_0 + pixel[9]) < c_b)
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[6]) < c_b)
                        if(*(cache_0 + pixel[7]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[6]) < c_b)
                       if(*(cache_0 + pixel[5]) < c_b)
                        goto success;
                       else
                        if(*(cache_0 + pixel[14]) < c_b)
                         if(*(cache_0 + pixel[15]) < c_b)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[14]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[14]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + pixel[6]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[5]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               if(*(cache_0 + -3) < c_b)
                if(*(cache_0 + pixel[10]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + pixel[11]) < c_b)
                   if(*(cache_0 + pixel[13]) < c_b)
                    if(*(cache_0 + pixel[15]) < c_b)
                     if(*(cache_0 + pixel[9]) > cb)
                      if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[2]) < c_b)
                        if(*(cache_0 + pixel[3]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else if(*(cache_0 + pixel[9]) < c_b)
                      if(*(cache_0 + pixel[1]) < c_b)
                       goto success;
                      else
                       if(*(cache_0 + pixel[7]) < c_b)
                        goto success;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[3]) < c_b)
                       if(*(cache_0 + pixel[2]) < c_b)
                        if(*(cache_0 + pixel[1]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     if(*(cache_0 + pixel[5]) < c_b)
                      if(*(cache_0 + pixel[6]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[9]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                continue;
             else
              if(*(cache_0 + pixel[2]) < c_b)
               if(*(cache_0 + -3) > cb)
                if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + pixel[7]) > cb)
                   if(*(cache_0 + pixel[1]) < c_b)
                    if(*(cache_0 + pixel[3]) < c_b)
                     if(*(cache_0 + 3) < c_b)
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[13]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else if(*(cache_0 + pixel[7]) < c_b)
                   if(*(cache_0 + 3) < c_b)
                    if(*(cache_0 + pixel[5]) < c_b)
                     if(*(cache_0 + pixel[1]) < c_b)
                      if(*(cache_0 + pixel[3]) < c_b)
                       if(*(cache_0 + pixel[15]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + pixel[13]) < c_b)
                    if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + 3) < c_b)
                       if(*(cache_0 + pixel[5]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + -3) < c_b)
                if(*(cache_0 + pixel[3]) > cb)
                 if(*(cache_0 + pixel[9]) < c_b)
                  if(*(cache_0 + pixel[11]) < c_b)
                   if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[15]) < c_b)
                      if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[10]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[3]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + 3) > cb)
                   if(*(cache_0 + pixel[10]) < c_b)
                    if(*(cache_0 + pixel[15]) < c_b)
                     if(*(cache_0 + pixel[1]) < c_b)
                      if(*(cache_0 + pixel[11]) < c_b)
                       if(*(cache_0 + pixel[13]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else if(*(cache_0 + 3) < c_b)
                   if(*(cache_0 + pixel[15]) < c_b)
                    if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[13]) > cb)
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[6]) < c_b)
                        if(*(cache_0 + pixel[7]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else if(*(cache_0 + pixel[13]) < c_b)
                      if(*(cache_0 + pixel[5]) < c_b)
                       goto success;
                      else
                       if(*(cache_0 + pixel[11]) < c_b)
                        goto success;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[6]) < c_b)
                        if(*(cache_0 + pixel[5]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   if(*(cache_0 + pixel[10]) < c_b)
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + pixel[15]) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[1]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                 else
                  continue;
                else
                 if(*(cache_0 + pixel[9]) < c_b)
                  if(*(cache_0 + pixel[10]) < c_b)
                   if(*(cache_0 + pixel[14]) < c_b)
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + pixel[15]) < c_b)
                      if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[13]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else
                if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + pixel[14]) < c_b)
                  if(*(cache_0 + 3) < c_b)
                   if(*(cache_0 + pixel[13]) > cb)
                    if(*(cache_0 + pixel[7]) < c_b)
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[5]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[13]) < c_b)
                    if(*(cache_0 + pixel[5]) < c_b)
                     if(*(cache_0 + pixel[15]) < c_b)
                      if(*(cache_0 + pixel[1]) < c_b)
                       if(*(cache_0 + pixel[3]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[7]) < c_b)
                     if(*(cache_0 + pixel[15]) < c_b)
                      if(*(cache_0 + pixel[3]) < c_b)
                       if(*(cache_0 + pixel[5]) < c_b)
                        if(*(cache_0 + pixel[1]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
            else
             if(*(cache_0 + pixel[8]) > cb)
              if(*(cache_0 + pixel[10]) > cb)
               if(*(cache_0 + 3) > cb)
                if(*(cache_0 + pixel[2]) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + pixel[7]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    if(*(cache_0 + pixel[9]) > cb)
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + pixel[3]) > cb)
                       goto success;
                      else if(*(cache_0 + pixel[3]) < c_b)
                       if(*(cache_0 + -3) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + -3) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else if(*(cache_0 + pixel[5]) < c_b)
                      if(*(cache_0 + -3) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      if(*(cache_0 + pixel[15]) > cb)
                       if(*(cache_0 + pixel[14]) > cb)
                        if(*(cache_0 + -3) > cb)
                         if(*(cache_0 + pixel[13]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[1]) > cb)
                     if(*(cache_0 + pixel[3]) > cb)
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + pixel[9]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else if(*(cache_0 + pixel[2]) < c_b)
                 if(*(cache_0 + pixel[11]) > cb)
                  if(*(cache_0 + -3) > cb)
                   if(*(cache_0 + pixel[9]) > cb)
                    if(*(cache_0 + pixel[6]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[13]) > cb)
                       if(*(cache_0 + pixel[5]) > cb)
                        goto success;
                       else
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else
                       if(*(cache_0 + pixel[3]) > cb)
                        if(*(cache_0 + pixel[5]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[6]) > cb)
                   if(*(cache_0 + pixel[11]) > cb)
                    if(*(cache_0 + pixel[13]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[5]) > cb)
                        goto success;
                       else if(*(cache_0 + pixel[5]) < c_b)
                        if(*(cache_0 + pixel[14]) > cb)
                         if(*(cache_0 + pixel[15]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                       else
                        if(*(cache_0 + pixel[15]) > cb)
                         if(*(cache_0 + pixel[14]) > cb)
                          goto success;
                         else
                          continue;
                        else
                         continue;
                      else
                       continue;
                     else
                      continue;
                    else if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[3]) > cb)
                      if(*(cache_0 + pixel[5]) > cb)
                       if(*(cache_0 + pixel[7]) > cb)
                        if(*(cache_0 + pixel[9]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + pixel[3]) > cb)
                      if(*(cache_0 + pixel[7]) > cb)
                       if(*(cache_0 + pixel[9]) > cb)
                        if(*(cache_0 + pixel[5]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
               else if(*(cache_0 + 3) < c_b)
                if(*(cache_0 + pixel[6]) > cb)
                 if(*(cache_0 + pixel[14]) > cb)
                  if(*(cache_0 + pixel[13]) > cb)
                   if(*(cache_0 + pixel[7]) > cb)
                    if(*(cache_0 + pixel[15]) > cb)
                     if(*(cache_0 + pixel[9]) > cb)
                      if(*(cache_0 + pixel[11]) > cb)
                       if(*(cache_0 + -3) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[11]) > cb)
                        if(*(cache_0 + -3) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else
                if(*(cache_0 + pixel[14]) > cb)
                 if(*(cache_0 + pixel[6]) > cb)
                  if(*(cache_0 + -3) > cb)
                   if(*(cache_0 + pixel[5]) > cb)
                    if(*(cache_0 + pixel[11]) > cb)
                     if(*(cache_0 + pixel[9]) > cb)
                      if(*(cache_0 + pixel[7]) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[5]) < c_b)
                    if(*(cache_0 + pixel[15]) > cb)
                     if(*(cache_0 + pixel[7]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[11]) > cb)
                        if(*(cache_0 + pixel[13]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[15]) > cb)
                     if(*(cache_0 + pixel[11]) > cb)
                      if(*(cache_0 + pixel[9]) > cb)
                       if(*(cache_0 + pixel[13]) > cb)
                        if(*(cache_0 + pixel[7]) > cb)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else if(*(cache_0 + pixel[8]) < c_b)
              if(*(cache_0 + pixel[10]) < c_b)
               if(*(cache_0 + 3) > cb)
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + -3) < c_b)
                   if(*(cache_0 + pixel[9]) < c_b)
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + pixel[15]) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[13]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
               else if(*(cache_0 + 3) < c_b)
                if(*(cache_0 + pixel[6]) < c_b)
                 if(*(cache_0 + -3) > cb)
                  if(*(cache_0 + pixel[2]) < c_b)
                   if(*(cache_0 + pixel[1]) > cb)
                    if(*(cache_0 + pixel[3]) < c_b)
                     if(*(cache_0 + pixel[5]) < c_b)
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[9]) < c_b)
                        if(*(cache_0 + pixel[11]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[1]) < c_b)
                    if(*(cache_0 + pixel[5]) < c_b)
                     if(*(cache_0 + pixel[9]) < c_b)
                      if(*(cache_0 + pixel[3]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[9]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else if(*(cache_0 + -3) < c_b)
                  if(*(cache_0 + pixel[7]) < c_b)
                   if(*(cache_0 + pixel[11]) > cb)
                    if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[2]) < c_b)
                      if(*(cache_0 + pixel[3]) < c_b)
                       if(*(cache_0 + pixel[5]) < c_b)
                        if(*(cache_0 + pixel[9]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[11]) < c_b)
                    if(*(cache_0 + pixel[9]) < c_b)
                     if(*(cache_0 + pixel[5]) > cb)
                      if(*(cache_0 + pixel[13]) < c_b)
                       if(*(cache_0 + pixel[14]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else if(*(cache_0 + pixel[5]) < c_b)
                      if(*(cache_0 + pixel[13]) < c_b)
                       goto success;
                      else
                       if(*(cache_0 + pixel[3]) < c_b)
                        goto success;
                       else
                        continue;
                     else
                      if(*(cache_0 + pixel[15]) < c_b)
                       if(*(cache_0 + pixel[14]) < c_b)
                        if(*(cache_0 + pixel[13]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[1]) < c_b)
                     if(*(cache_0 + pixel[2]) < c_b)
                      if(*(cache_0 + pixel[9]) < c_b)
                       if(*(cache_0 + pixel[3]) < c_b)
                        if(*(cache_0 + pixel[5]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  if(*(cache_0 + pixel[2]) < c_b)
                   if(*(cache_0 + pixel[1]) < c_b)
                    if(*(cache_0 + pixel[3]) < c_b)
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[9]) < c_b)
                       if(*(cache_0 + pixel[5]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[11]) < c_b)
                     if(*(cache_0 + pixel[3]) < c_b)
                      if(*(cache_0 + pixel[5]) < c_b)
                       if(*(cache_0 + pixel[7]) < c_b)
                        if(*(cache_0 + pixel[9]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                else
                 continue;
               else
                if(*(cache_0 + pixel[14]) < c_b)
                 if(*(cache_0 + pixel[6]) < c_b)
                  if(*(cache_0 + -3) < c_b)
                   if(*(cache_0 + pixel[5]) > cb)
                    if(*(cache_0 + pixel[9]) < c_b)
                     if(*(cache_0 + pixel[7]) < c_b)
                      if(*(cache_0 + pixel[11]) < c_b)
                       if(*(cache_0 + pixel[13]) < c_b)
                        if(*(cache_0 + pixel[15]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else if(*(cache_0 + pixel[5]) < c_b)
                    if(*(cache_0 + pixel[13]) < c_b)
                     if(*(cache_0 + pixel[11]) < c_b)
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[9]) < c_b)
                        goto success;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                   else
                    if(*(cache_0 + pixel[15]) < c_b)
                     if(*(cache_0 + pixel[13]) < c_b)
                      if(*(cache_0 + pixel[7]) < c_b)
                       if(*(cache_0 + pixel[9]) < c_b)
                        if(*(cache_0 + pixel[11]) < c_b)
                         goto success;
                        else
                         continue;
                       else
                        continue;
                      else
                       continue;
                     else
                      continue;
                    else
                     continue;
                  else
                   continue;
                 else
                  continue;
                else
                 continue;
              else
               continue;
             else
              continue;

            success:
                    corners.push_back(fast_xy(static_cast<short>(cache_0-line_min),
                                              static_cast<short>(y)));
    }
  }
}

} // namespace Fast
