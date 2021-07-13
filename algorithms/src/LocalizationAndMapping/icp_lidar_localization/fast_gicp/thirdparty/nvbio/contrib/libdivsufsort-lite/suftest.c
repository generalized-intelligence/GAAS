/*
 * suftest.c for divsufsort-lite
 * Copyright (c) 2003-2008 Yuta Mori All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "divsufsort.h"


/* Checks the suffix array SA of the string T. */
int
sufcheck(const unsigned char *T, const int *SA, int n, int verbose) {
  int C[256];
  int i, p, q, t;
  int c;

  if(verbose) { fprintf(stderr, "sufcheck: "); }
  if(n == 0) {
    if(verbose) { fprintf(stderr, "Done.\n"); }
    return 0;
  }

  /* Check arguments. */
  if((T == NULL) || (SA == NULL) || (n < 0)) {
    if(verbose) { fprintf(stderr, "Invalid arguments.\n"); }
    return -1;
  }

  /* check range: [0..n-1] */
  for(i = 0; i < n; ++i) {
    if((SA[i] < 0) || (n <= SA[i])) {
      if(verbose) {
        fprintf(stderr, "Out of the range [0,%d].\n"
                        "  SA[%d]=%d\n",
                        n - 1, i, SA[i]);
      }
      return -2;
    }
  }

  /* check first characters. */
  for(i = 1; i < n; ++i) {
    if(T[SA[i - 1]] > T[SA[i]]) {
      if(verbose) {
        fprintf(stderr, "Suffixes in wrong order.\n"
                        "  T[SA[%d]=%d]=%d > T[SA[%d]=%d]=%d\n",
                        i - 1, SA[i - 1], T[SA[i - 1]], i, SA[i], T[SA[i]]);
      }
      return -3;
    }
  }

  /* check suffixes. */
  for(i = 0; i < 256; ++i) { C[i] = 0; }
  for(i = 0; i < n; ++i) { ++C[T[i]]; }
  for(i = 0, p = 0; i < 256; ++i) {
    t = C[i];
    C[i] = p;
    p += t;
  }

  q = C[T[n - 1]];
  C[T[n - 1]] += 1;
  for(i = 0; i < n; ++i) {
    p = SA[i];
    if(0 < p) {
      c = T[--p];
      t = C[c];
    } else {
      c = T[p = n - 1];
      t = q;
    }
    if((t < 0) || (p != SA[t])) {
      if(verbose) {
        fprintf(stderr, "Suffix in wrong position.\n"
                        "  SA[%d]=%d or\n"
                        "  SA[%d]=%d\n",
                        t, (0 <= t) ? SA[t] : -1, i, SA[i]);
      }
      return -4;
    }
    if(t != q) {
      ++C[c];
      if((n <= C[c]) || (T[SA[C[c]]] != c)) { C[c] = -1; }
    }
  }

  if(1 <= verbose) { fprintf(stderr, "Done.\n"); }
  return 0;
}

static
void
print_help(const char *progname, int status) {
  fprintf(stderr, "usage: %s FILE\n\n", progname);
  exit(status);
}

int
main(int argc, const char *argv[]) {
  FILE *fp;
  const char *fname;
  unsigned char *T;
  int *SA;
  long n;
  clock_t start, finish;

  /* Check arguments. */
  if((argc == 1) ||
     (strcmp(argv[1], "-h") == 0) ||
     (strcmp(argv[1], "--help") == 0)) { print_help(argv[0], EXIT_SUCCESS); }
  if(argc != 2) { print_help(argv[0], EXIT_FAILURE); }

  /* Open a file for reading. */
  if((fp = fopen(fname = argv[1], "rb")) == NULL) {
    fprintf(stderr, "%s: Cannot open file `%s': ", argv[0], fname);
    perror(NULL);
    exit(EXIT_FAILURE);
  }

  /* Get the file size. */
  if(fseek(fp, 0, SEEK_END) == 0) {
    n = ftell(fp);
    rewind(fp);
    if(n < 0) {
      fprintf(stderr, "%s: Cannot ftell `%s': ", argv[0], fname);
      perror(NULL);
      exit(EXIT_FAILURE);
    }
  } else {
    fprintf(stderr, "%s: Cannot fseek `%s': ", argv[0], fname);
    perror(NULL);
    exit(EXIT_FAILURE);
  }

  /* Allocate 5n bytes of memory. */
  T = (unsigned char *)malloc((size_t)n * sizeof(unsigned char));
  SA = (int *)malloc((size_t)n * sizeof(int));
  if((T == NULL) || (SA == NULL)) {
    fprintf(stderr, "%s: Cannot allocate memory.\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  /* Read n bytes of data. */
  if(fread(T, sizeof(unsigned char), (size_t)n, fp) != (size_t)n) {
    fprintf(stderr, "%s: %s `%s': ",
      argv[0],
      (ferror(fp) || !feof(fp)) ? "Cannot read from" : "Unexpected EOF in",
      argv[1]);
    perror(NULL);
    exit(EXIT_FAILURE);
  }
  fclose(fp);

  /* Construct the suffix array. */
  fprintf(stderr, "%s: %ld bytes ... ", fname, n);
  start = clock();
  if(divsufsort(T, SA, (int)n) != 0) {
    fprintf(stderr, "%s: Cannot allocate memory.\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  finish = clock();
  fprintf(stderr, "%.4f sec\n", (double)(finish - start) / (double)CLOCKS_PER_SEC);

  /* Check the suffix array. */
  if(sufcheck(T, SA, (int)n, 1) != 0) { exit(EXIT_FAILURE); }

  /* Deallocate memory. */
  free(SA);
  free(T);

  return 0;
}
