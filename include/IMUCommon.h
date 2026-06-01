// Common IMU utilities: covariance accumulator and helpers
// This header provides a small, header-only utility to accumulate 3-axis samples
// and compute a 3x3 covariance matrix in row-major order.
// The intent is to reduce duplicated code across IMU drivers and provide a
// consistent, well-documented API for covariance calculation.

#ifndef IMU_COMMON_H
#define IMU_COMMON_H

#include <Arduino.h>
#include <string.h> // memset
#include <new>      // std::nothrow for safe allocation

struct CovarianceAccumulator
{
    // This accumulator supports two modes:
    //  - unlimited accumulation (default): behavior compatible with the previous implementation
    //  - rolling-window accumulation: keep only the most recent N samples (call setWindowSize(N) to enable)
    //
    // Formulas used (unbiased sample covariance):
    //   mean_x = (1/n) * Σ x_i
    //   cov_xy = (1/(n-1)) * Σ_i (x_i - mean_x) (y_i - mean_y)
    // which can be computed from sums as:
    //   Σ (x_i - mean_x)(y_i - mean_y) = Σ x_i y_i - n * mean_x * mean_y
    // and then divide by (n-1) for the unbiased estimator.

    // Accumulators for sums and cross-products
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
    float sumXX = 0.0f;
    float sumYY = 0.0f;
    float sumZZ = 0.0f;
    float sumXY = 0.0f;
    float sumXZ = 0.0f;
    float sumYZ = 0.0f;
    int sampleCount = 0; // current number of samples accounted for (<= windowSize if windowing enabled)

    // Rolling buffer (optional). If windowSize == 0, no buffer is used and all samples accumulate.
    float *bufX = nullptr;
    float *bufY = nullptr;
    float *bufZ = nullptr;
    int windowSize = 0; // 0 means unlimited
    int bufHead = 0;    // next write index

    // Minimum variance epsilon: clamp very small or negative variances to 0 or this epsilon
    float varianceEpsilon = 1e-9f;

    // IMPORTANT: For long-running systems (days+), consider calling reset() periodically to prevent
    // numerical precision loss in the sum accumulators. Set a timer in your application logic.
    // With rolling window mode (recommended), precision is automatically bounded to the window size.

    // Configure a rolling window. Passing size <= 0 disables windowing and restores unlimited accumulation.
    // Calling this will reset any existing accumulated data.
    // IMPORTANT: On embedded systems, allocation may fail if RAM is exhausted.
    // This method will safely fall back to unlimited accumulation if allocation fails.
    void setWindowSize(int size)
    {
        // free existing buffers
        if (bufX) { delete[] bufX; bufX = nullptr; }
        if (bufY) { delete[] bufY; bufY = nullptr; }
        if (bufZ) { delete[] bufZ; bufZ = nullptr; }

        if (size > 0)
        {
            // Attempt to allocate buffer memory. If allocation fails on embedded systems
            // with limited RAM, gracefully fall back to unlimited accumulation mode.
            float *tempX = new (std::nothrow) float[size];
            float *tempY = new (std::nothrow) float[size];
            float *tempZ = new (std::nothrow) float[size];

            // Check if ALL three allocations succeeded before committing
            if (tempX && tempY && tempZ)
            {
                windowSize = size;
                bufX = tempX;
                bufY = tempY;
                bufZ = tempZ;
            }
            else
            {
                // Allocation failed: fall back to unlimited accumulation
                delete[] tempX;
                delete[] tempY;
                delete[] tempZ;
                windowSize = 0;
                bufX = bufY = bufZ = nullptr;
            }
        }
        else
        {
            windowSize = 0;
        }
        reset();
    }

    void setVarianceEpsilon(float eps)
    {
        if (eps >= 0.0f) varianceEpsilon = eps;
    }

    // Reset all accumulators and buffer state
    void reset()
    {
        sumX = sumY = sumZ = 0.0f;
        sumXX = sumYY = sumZZ = 0.0f;
        sumXY = sumXZ = sumYZ = 0.0f;
        sampleCount = 0;
        bufHead = 0;
        if (windowSize > 0)
        {
            // initialize buffer contents to zero
            for (int i = 0; i < windowSize; ++i)
            {
                bufX[i] = bufY[i] = bufZ[i] = 0.0f;
            }
        }
    }

    // Add a single 3-axis sample (x,y,z)
    // If a rolling window is configured, the oldest sample is removed when the buffer is full.
    // If buffer allocation failed, falls back to unlimited accumulation safely.
    void addSample(float x, float y, float z)
    {
        if (windowSize <= 0 || bufX == nullptr)
        {
            // unlimited accumulation (no window, or allocation failed earlier)
            sumX += x;
            sumY += y;
            sumZ += z;
            sumXX += x * x;
            sumYY += y * y;
            sumZZ += z * z;
            sumXY += x * y;
            sumXZ += x * z;
            sumYZ += y * z;
            sampleCount++;
            return;
        }

        // rolling window: overwrite at bufHead
        if (sampleCount < windowSize)
        {
            // buffer not yet full: just add
            bufX[bufHead] = x;
            bufY[bufHead] = y;
            bufZ[bufHead] = z;

            sumX += x;
            sumY += y;
            sumZ += z;
            sumXX += x * x;
            sumYY += y * y;
            sumZZ += z * z;
            sumXY += x * y;
            sumXZ += x * z;
            sumYZ += y * z;

            sampleCount++;
            bufHead = (bufHead + 1) % windowSize;
            return;
        }

        // buffer full: subtract oldest sample at bufHead then add new sample
        float oldX = bufX[bufHead];
        float oldY = bufY[bufHead];
        float oldZ = bufZ[bufHead];

        // remove old contributions
        sumX -= oldX;
        sumY -= oldY;
        sumZ -= oldZ;
        sumXX -= oldX * oldX;
        sumYY -= oldY * oldY;
        sumZZ -= oldZ * oldZ;
        sumXY -= oldX * oldY;
        sumXZ -= oldX * oldZ;
        sumYZ -= oldY * oldZ;

        // write new
        bufX[bufHead] = x;
        bufY[bufHead] = y;
        bufZ[bufHead] = z;

        // add new contributions
        sumX += x;
        sumY += y;
        sumZ += z;
        sumXX += x * x;
        sumYY += y * y;
        sumZZ += z * z;
        sumXY += x * y;
        sumXZ += x * z;
        sumYZ += y * z;

        bufHead = (bufHead + 1) % windowSize;
        // sampleCount remains equal to windowSize when full
    }

    // Compute the 3x3 covariance matrix (row-major) into outMatrix (must have 9 elements).
    // Uses the unbiased estimator (divide by n-1) when n >= 2. If fewer than 2 samples,
    // the matrix is zeroed. Small negative variances (due to numerical noise) are clamped to 0
    // and any variance smaller than varianceEpsilon is clamped to 0 as well.
    void computeCovMatrix(float outMatrix[9]) const
    {
        if (sampleCount < 2)
        {
            memset(outMatrix, 0, sizeof(float) * 9);
            return;
        }

        const float n = static_cast<float>(sampleCount);
        const float meanX = sumX / n;
        const float meanY = sumY / n;
        const float meanZ = sumZ / n;

        // Numerator = Σ x*y - n * mean_x * mean_y
        const float numXX = (sumXX) - (n * meanX * meanX);
        const float numYY = (sumYY) - (n * meanY * meanY);
        const float numZZ = (sumZZ) - (n * meanZ * meanZ);
        const float numXY = (sumXY) - (n * meanX * meanY);
        const float numXZ = (sumXZ) - (n * meanX * meanZ);
        const float numYZ = (sumYZ) - (n * meanY * meanZ);

        const float denom = (n - 1.0f); // unbiased estimator
        outMatrix[0] = numXX / denom;
        outMatrix[1] = numXY / denom;
        outMatrix[2] = numXZ / denom;
        outMatrix[3] = outMatrix[1];
        outMatrix[4] = numYY / denom;
        outMatrix[5] = numYZ / denom;
        outMatrix[6] = outMatrix[2];
        outMatrix[7] = outMatrix[5];
        outMatrix[8] = numZZ / denom;

        // Clamp tiny negative values (numerical noise) and values below epsilon
        for (int i : {0, 4, 8})
        {
            if (outMatrix[i] < 0.0f) outMatrix[i] = 0.0f;
            if (outMatrix[i] < varianceEpsilon) outMatrix[i] = 0.0f;
        }
    }

    // Return number of samples currently accumulated (useful when windowing)
    int getSampleCount() const { return sampleCount; }

    // Check if rolling window was successfully allocated (for diagnostics).
    // Returns: true if window is active, false if unlimited accumulation mode or allocation failed.
    bool isWindowValid() const { return (windowSize > 0 && bufX != nullptr && bufY != nullptr && bufZ != nullptr); }
};

#endif // IMU_COMMON_H
