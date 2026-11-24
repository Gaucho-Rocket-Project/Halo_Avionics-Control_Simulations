#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static inline void printmatrix(int r, int c, const float *a) {
  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {
      printf("%10.6f ", a[i * c + j]);
    }
    printf("\n");
  }
}

static inline void matmul(int n, int m, int p, const float *a, const float *b,
                   float *product) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < p; j++) {
      float sum = 0.0f;
      for (int k = 0; k < m; k++) {
        sum += a[i * m + k] * b[k * p + j];
      }
      product[i * p + j] = sum;
    }
  }
}

static inline void matadd(int r, int c, const float *a, const float *b, float *sum) {
  for (int i = 0; i < r * c; i++) {
    sum[i] = a[i] + b[i];
  }
}

static inline void matsub(int r, int c, const float *a, const float *b,
                          float *difference) {
  for (int i = 0; i < r * c; i++) {
    difference[i] = a[i] - b[i];
  }
}

static inline void identity(int n, float *out) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      out[i * n + j] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

static inline void transpose(int r, int c, const float *a, float *out) {
  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {
      out[j * r + i] = a[i * c + j];
    }
  }
}

/* out = inverse of in (both n×n). work is scratch space of n*n floats.
 * Returns false if the matrix is singular. out may alias in. */
static inline bool matinv(int n, const float *in, float *out, float *work) {
  /* work = copy of in, out = identity */
  for (int i = 0; i < n * n; i++) {
    work[i] = in[i];
  }
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      out[i * n + j] = (i == j) ? 1.0f : 0.0f;
    }
  }

  for (int col = 0; col < n; col++) {
    /* partial pivoting: pick the row with the largest entry in this column */
    int pivot = col;
    for (int r = col + 1; r < n; r++) {
      if (fabsf(work[r * n + col]) > fabsf(work[pivot * n + col])) {
        pivot = r;
      }
    }
    if (fabsf(work[pivot * n + col]) < 1e-12f) {
      return false; /* singular */
    }

    /* swap the pivot row into place (in both matrices) */
    if (pivot != col) {
      for (int j = 0; j < n; j++) {
        float t = work[col * n + j];
        work[col * n + j] = work[pivot * n + j];
        work[pivot * n + j] = t;

        t = out[col * n + j];
        out[col * n + j] = out[pivot * n + j];
        out[pivot * n + j] = t;
      }
    }

    /* scale the pivot row so the pivot becomes 1 */
    float inv_p = 1.0f / work[col * n + col];
    for (int j = 0; j < n; j++) {
      work[col * n + j] *= inv_p;
      out[col * n + j] *= inv_p;
    }

    /* eliminate this column from every other row */
    for (int r = 0; r < n; r++) {
      if (r == col)
        continue;
      float f = work[r * n + col];
      for (int j = 0; j < n; j++) {
        work[r * n + j] -= f * work[col * n + j];
        out[r * n + j] -= f * out[col * n + j];
      }
    }
  }
  return true;
}
