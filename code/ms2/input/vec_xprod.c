const int jBlk = 16;

// Function to compute the cross product of two 3D vectors
inline void crossProduct(int *a, int *b, int *result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// Function to compute the Biot-Savart law cross product components
void computeBiotSavartCrossProduct(int N, int uwv[][3], int xyz[][3], int result[][3]) {
    int r[3]; // Distance vector
    int temp_result[3]; // Temporary result vector for accumulation

    for (int jj = 0; jj < N; jj += jBlk) {
        for (int i = 0; i < N; i++) {
            // Initialize temporary result accumulators to zero
            temp_result[0] = temp_result[1] = temp_result[2] = 0;

            for( int j = jj; j < jj + jBlk; j++ ) {
                if( i != j ) { // Avoid computing self-interaction
                    // Compute the distance vector r = xyz[j] - xyz[i]
                    r[0] = xyz[j][0] - xyz[i][0];
                    r[1] = xyz[j][1] - xyz[i][1];
                    r[2] = xyz[j][2] - xyz[i][2];

                    // Compute the cross product of uwv[i] and r
                    int cross[3];
                    crossProduct( uwv[i], r, cross );

                    // Accumulate the cross product result into the temporary result array
                    temp_result[0] += cross[0];
                    temp_result[1] += cross[1];
                    temp_result[2] += cross[2];
                }
            }

            // Store the accumulated temporary result into the result array
            result[i][0] += temp_result[0];
            result[i][1] += temp_result[1];
            result[i][2] += temp_result[2];
        }
    }
}