/**
 * @file FusionTypes.h
 * @author Seb Madgwick
 * @brief Common types and their associated operations.
 *
 * Static inline implementations are used to optimise for increased execution
 * speed.
 */

#ifndef FUSION_TYPES_H
#define FUSION_TYPES_H

//------------------------------------------------------------------------------
// Includes

#include <math.h>   // M_PI, sqrtf, atan2f, asinf
#include <stdint.h> // int32_t

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Three-dimensional spacial vector.
 */
typedef union {
    float array[3];

    struct {
        float x;
        float y;
        float z;
    } axis;
} fusion_vector3_t;

/**
 * @brief Quaternion.  This library uses the conversion of placing the 'w'
 * element as the first element.  Other implementations may place the 'w'
 * element as the last element.
 */
typedef union {
    float array[4];

    struct {
        float w;
        float x;
        float y;
        float z;
    } element;
} fusion_quaternion_t;

/**
 * @brief Rotation matrix in row-major order.
 * See http://en.wikipedia.org/wiki/Row-major_order
 */
typedef union {
    float array[9];

    struct {
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    } element;
} fusion_rotation_matrix_t;

/**
 * @brief Euler angles union.  The Euler angles are in the Aerospace sequence
 * also known as the ZYX sequence.
 */
typedef union {
    float array[3];

    struct {
        float roll;
        float pitch;
        float yaw;
    } angle;
} fusion_euler_angles_t;

/**
 * @brief Zero-length vector definition.
 */
#define FUSION_VECTOR3_ZERO ((fusion_vector3_t){.array = {0.0f, 0.0f, 0.0f}})

/**
 * @brief Quaternion identity definition to represent an aligned of
 * orientation.
 */
#define FUSION_QUATERNION_IDENTITY ((fusion_quaternion_t){.array = {1.0f, 0.0f, 0.0f, 0.0f}})

/**
 * @brief Rotation matrix identity definition to represent an aligned of
 * orientation.
 */
#define FUSION_ROTATION_MATRIX_IDENTITY \
    ((fusion_rotation_matrix_t){.array = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}})

/**
 * @brief Euler angles zero definition to represent an aligned of orientation.
 */
#define FUSION_EULER_ANGLES_ZERO ((fusion_euler_angles_t){.array = {0.0f, 0.0f, 0.0f}})

/**
 * @brief Definition of M_PI.  Some compilers may not define this in math.h.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//------------------------------------------------------------------------------
// Inline functions - Degrees and radians conversion

/**
 * @brief Converts degrees to radians.
 * @param degrees Degrees.
 * @return Radians.
 */
static inline __attribute__((always_inline)) float fusion_degrees_to_radians(const float degrees)
{
    return degrees * ((float)M_PI / 180.0f);
}

/**
 * @brief Converts radians to degrees.
 * @param radians Radians.
 * @return Degrees.
 */
static inline __attribute__((always_inline)) float fusion_radians_to_degrees(const float radians)
{
    return radians * (180.0f / (float)M_PI);
}

//------------------------------------------------------------------------------
// Inline functions - Fast inverse square root

/**
 * @brief Calculates the reciprocal of the square root.
 * See http://en.wikipedia.org/wiki/Fast_inverse_square_root
 * @param x Operand.
 * @return Reciprocal of the square root of x.
 */
static inline __attribute__((always_inline)) float fusion_fast_inverse_sqrt(const float x)
{
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//------------------------------------------------------------------------------
// Inline functions - Vector operations

/**
 * @brief Adds two vectors.
 * @param vectorA First vector of the operation.
 * @param vectorB Second vector of the operation.
 * @return Sum of vectorA and vectorB.
 */
static inline __attribute__((always_inline)) fusion_vector3_t
fusion_vector_add(const fusion_vector3_t vectorA, const fusion_vector3_t vectorB)
{
    fusion_vector3_t result;
    result.axis.x = vectorA.axis.x + vectorB.axis.x;
    result.axis.y = vectorA.axis.y + vectorB.axis.y;
    result.axis.z = vectorA.axis.z + vectorB.axis.z;
    return result;
}

/**
 * @brief Subtracts two vectors.
 * @param vectorA First vector of the operation.
 * @param vectorB Second vector of the operation.
 * @return vectorB subtracted from vectorA.
 */
static inline __attribute__((always_inline)) fusion_vector3_t
fusion_vector_subtract(const fusion_vector3_t vectorA, const fusion_vector3_t vectorB)
{
    fusion_vector3_t result;
    result.axis.x = vectorA.axis.x - vectorB.axis.x;
    result.axis.y = vectorA.axis.y - vectorB.axis.y;
    result.axis.z = vectorA.axis.z - vectorB.axis.z;
    return result;
}

/**
 * @brief Multiplies vector by a scalar.
 * @param vector Vector to be multiplied.
 * @param scalar Scalar to be multiplied.
 * @return Vector multiplied by scalar.
 */
static inline __attribute__((always_inline)) fusion_vector3_t
fusion_vector_multiply_scalar(const fusion_vector3_t vector, const float scalar)
{
    fusion_vector3_t result;
    result.axis.x = vector.axis.x * scalar;
    result.axis.y = vector.axis.y * scalar;
    result.axis.z = vector.axis.z * scalar;
    return result;
}

/**
 * @brief Calculates the Hadamard product (element-wise multiplication) of two
 * vectors.
 * @param vectorA First vector of the operation.
 * @param vectorB Second vector of the operation.
 * @return Hadamard product of vectorA and vectorB.
 */
static inline __attribute__((always_inline)) fusion_vector3_t
fusion_vector_hadamard_product(const fusion_vector3_t vectorA, const fusion_vector3_t vectorB)
{
    fusion_vector3_t result;
    result.axis.x = vectorA.axis.x * vectorB.axis.x;
    result.axis.y = vectorA.axis.y * vectorB.axis.y;
    result.axis.z = vectorA.axis.z * vectorB.axis.z;
    return result;
}

/**
 * @brief Calculates the cross-product of two vectors.
 * @param vectorA First vector of the operation.
 * @param vectorB Second vector of the operation.
 * @return Cross-product of vectorA and vectorB.
 */
static inline __attribute__((always_inline)) fusion_vector3_t
fusion_vector_cross_product(const fusion_vector3_t vectorA, const fusion_vector3_t vectorB)
{
#define A vectorA.axis // define shorthand labels for more readable code
#define B vectorB.axis
    fusion_vector3_t result;
    result.axis.x = A.y * B.z - A.z * B.y;
    result.axis.y = A.z * B.x - A.x * B.z;
    result.axis.z = A.x * B.y - A.y * B.x;
    return result;
#undef A // undefine shorthand labels
#undef B
}

/**
 * @brief Calculates the vector magnitude squared.
 * @param vector Vector of the operation.
 * @return Vector magnitude squared.
 */
static inline __attribute__((always_inline)) float fusion_vector_magnitude_squared(const fusion_vector3_t vector)
{
#define V vector.axis // define shorthand label for more readable code
    return V.x * V.x + V.y * V.y + V.z * V.z;
#undef V // undefine shorthand label
}

/**
 * @brief Calculates the magnitude of a vector.
 * @param vector Vector to be used in calculation.
 * @return Vector magnitude.
 */
static inline __attribute__((always_inline)) float fusion_vector_magnitude(const fusion_vector3_t vector)
{
    return sqrtf(fusion_vector_magnitude_squared(vector));
}

/**
 * @brief Normalises a vector to be of unit magnitude.
 * @param vector Vector to be normalised.
 * @return Normalised vector.
 */
static inline __attribute__((always_inline)) fusion_vector3_t fusion_vector_normalise(const fusion_vector3_t vector)
{
    const float magnitudeReciprocal = 1.0f / sqrtf(fusion_vector_magnitude_squared(vector));
    return fusion_vector_multiply_scalar(vector, magnitudeReciprocal);
}

/**
 * @brief Normalises a vector to be of unit magnitude using the fast inverse
 * square root approximation.
 * @param vector Vector to be normalised.
 * @return Normalised vector.
 */
static inline __attribute__((always_inline)) fusion_vector3_t fusion_vector_fast_normalise(const fusion_vector3_t vector)
{
    const float magnitudeReciprocal = fusion_fast_inverse_sqrt(fusion_vector_magnitude_squared(vector));
    return fusion_vector_multiply_scalar(vector, magnitudeReciprocal);
}

//------------------------------------------------------------------------------
// Inline functions - Quaternion operations

/**
 * @brief Adds two quaternions.
 * @param quaternionA First quaternion of the operation.
 * @param quaternionB Second quaternion of the operation.
 * @return Sum of quaternionA and quaternionB.
 */
static inline __attribute__((always_inline)) fusion_quaternion_t
fusion_quaternion_add(const fusion_quaternion_t quaternionA, const fusion_quaternion_t quaternionB)
{
    fusion_quaternion_t result;
    result.element.w = quaternionA.element.w + quaternionB.element.w;
    result.element.x = quaternionA.element.x + quaternionB.element.x;
    result.element.y = quaternionA.element.y + quaternionB.element.y;
    result.element.z = quaternionA.element.z + quaternionB.element.z;
    return result;
}

/**
 * @brief Multiplies two quaternions.
 * @param quaternionA First quaternion of the operation.
 * @param quaternionB Second quaternion of the operation.
 * @return quaternionA multiplied by quaternionB.
 */
static inline __attribute__((always_inline)) fusion_quaternion_t
fusion_quaternion_multiply(const fusion_quaternion_t quaternionA, const fusion_quaternion_t quaternionB)
{
#define A quaternionA.element // define shorthand labels for more readable code
#define B quaternionB.element
    fusion_quaternion_t result;
    result.element.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
    result.element.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y;
    result.element.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
    result.element.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;
    return result;
#undef A // undefine shorthand labels
#undef B
}

/**
 * @brief Multiplies quaternion by a vector.  This is a normal quaternion
 * multiplication where the vector is treated a quaternion with a 'w' element
 * value of 0.  The quaternion is post multiplied by the vector.
 * @param quaternion Quaternion to be multiplied.
 * @param vector Vector to be multiplied.
 * @return Quaternion multiplied by vector.
 */
static inline __attribute__((always_inline)) fusion_quaternion_t
fusion_quaternion_multiply_vector(const fusion_quaternion_t quaternion, const fusion_vector3_t vector)
{
#define Q quaternion.element // define shorthand labels for more readable code
#define V vector.axis
    fusion_quaternion_t result;
    result.element.w = -Q.x * V.x - Q.y * V.y - Q.z * V.z;
    result.element.x = Q.w * V.x + Q.y * V.z - Q.z * V.y;
    result.element.y = Q.w * V.y - Q.x * V.z + Q.z * V.x;
    result.element.z = Q.w * V.z + Q.x * V.y - Q.y * V.x;
    return result;
#undef Q // undefine shorthand labels
#undef V
}

/**
 * @brief Returns the quaternion conjugate.
 * @param quaternion Quaternion to be conjugated.
 * @return Conjugated quaternion.
 */
static inline __attribute__((always_inline)) fusion_quaternion_t fusion_quaternion_conjugate(const fusion_quaternion_t quaternion)
{
    fusion_quaternion_t conjugate;
    conjugate.element.w = quaternion.element.w;
    conjugate.element.x = -1.0f * quaternion.element.x;
    conjugate.element.y = -1.0f * quaternion.element.y;
    conjugate.element.z = -1.0f * quaternion.element.z;
    return conjugate;
}

/**
 * @brief Normalises a quaternion to be of unit magnitude.
 * @param quaternion Quaternion to be normalised.
 * @return Normalised quaternion.
 */
static inline __attribute__((always_inline)) fusion_quaternion_t fusion_quaternion_normalise(const fusion_quaternion_t quaternion)
{
#define Q quaternion.element // define shorthand label for more readable code
    const float magnitudeReciprocal = 1.0f / sqrtf(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
    fusion_quaternion_t normalisedQuaternion;
    normalisedQuaternion.element.w = Q.w * magnitudeReciprocal;
    normalisedQuaternion.element.x = Q.x * magnitudeReciprocal;
    normalisedQuaternion.element.y = Q.y * magnitudeReciprocal;
    normalisedQuaternion.element.z = Q.z * magnitudeReciprocal;
    return normalisedQuaternion;
#undef Q // undefine shorthand label
}

/**
 * @brief Normalises a quaternion to be of unit magnitude using the fast inverse
 * square root approximation.
 * @param quaternion Quaternion to be normalised.
 * @return Normalised quaternion.
 */
static inline __attribute__((always_inline)) fusion_quaternion_t fusion_quaternion_fast_normalise(const fusion_quaternion_t quaternion)
{
#define Q quaternion.element // define shorthand label for more readable code
    const float magnitudeReciprocal = fusion_fast_inverse_sqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
    fusion_quaternion_t normalisedQuaternion;
    normalisedQuaternion.element.w = Q.w * magnitudeReciprocal;
    normalisedQuaternion.element.x = Q.x * magnitudeReciprocal;
    normalisedQuaternion.element.y = Q.y * magnitudeReciprocal;
    normalisedQuaternion.element.z = Q.z * magnitudeReciprocal;
    return normalisedQuaternion;
#undef Q // undefine shorthand label
}

//------------------------------------------------------------------------------
// Inline functions - Rotation matrix operations

/**
 * @brief Multiplies two rotation matrices.
 * @param rotationMatrixA First rotation matrix of the operation.
 * @param rotationMatrixB Second rotation matrix of the operation.
 * @return rotationMatrixA with rotationMatrixB.
 */
static inline __attribute__((always_inline)) fusion_rotation_matrix_t
fusion_rotation_matrix_multiply(const fusion_rotation_matrix_t rotationMatrixA, const fusion_rotation_matrix_t rotationMatrixB)
{
#define A rotationMatrixA.element // define shorthand label for more readable code
#define B rotationMatrixB.element
    fusion_rotation_matrix_t result;
    result.element.xx = A.xx * B.xx + A.xy * B.yx + A.xz * B.zx;
    result.element.xy = A.xx * B.xy + A.xy * B.yy + A.xz * B.zy;
    result.element.xz = A.xx * B.xz + A.xy * B.yz + A.xz * B.zz;
    result.element.yx = A.yx * B.xx + A.yy * B.yx + A.yz * B.zx;
    result.element.yy = A.yx * B.xy + A.yy * B.yy + A.yz * B.zy;
    result.element.yz = A.yx * B.xz + A.yy * B.yz + A.yz * B.zz;
    result.element.zx = A.zx * B.xx + A.zy * B.yx + A.zz * B.zx;
    result.element.zy = A.zx * B.xy + A.zy * B.yy + A.zz * B.zy;
    result.element.zz = A.zx * B.xz + A.zy * B.yz + A.zz * B.zz;
    return result;
#undef A // undefine shorthand label
#undef B
}

/**
 * @brief Multiplies rotation matrix with scalar.
 * @param rotationMatrix Rotation matrix to be multiplied.
 * @param vector Vector to be multiplied.
 * @return Rotation matrix multiplied with scalar.
 */
static inline __attribute__((always_inline)) fusion_vector3_t
fusion_rotation_matrix_multiply_vector(const fusion_rotation_matrix_t rotationMatrix, const fusion_vector3_t vector)
{
#define R rotationMatrix.element // define shorthand label for more readable code
    fusion_vector3_t result;
    result.axis.x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z;
    result.axis.y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z;
    result.axis.z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z;
    return result;
#undef R // undefine shorthand label
}

//------------------------------------------------------------------------------
// Inline functions - Conversion operations

/**
 * @brief Converts a quaternion to a rotation matrix.
 * @param quaternion Quaternion to be converted.
 * @return Rotation matrix.
 */
static inline __attribute__((always_inline)) fusion_rotation_matrix_t fusion_quaternion_to_rotation_matrix(
const fusion_quaternion_t quaternion)
{
#define Q quaternion.element      // define shorthand label for more readable code
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    fusion_rotation_matrix_t rotationMatrix;
    rotationMatrix.element.xx = 2.0f * (qwqw - 0.5f + Q.x * Q.x);
    rotationMatrix.element.xy = 2.0f * (qxqy + qwqz);
    rotationMatrix.element.xz = 2.0f * (qxqz - qwqy);
    rotationMatrix.element.yx = 2.0f * (qxqy - qwqz);
    rotationMatrix.element.yy = 2.0f * (qwqw - 0.5f + Q.y * Q.y);
    rotationMatrix.element.yz = 2.0f * (qyqz + qwqx);
    rotationMatrix.element.zx = 2.0f * (qxqz + qwqy);
    rotationMatrix.element.zy = 2.0f * (qyqz - qwqx);
    rotationMatrix.element.zz = 2.0f * (qwqw - 0.5f + Q.z * Q.z);
    return rotationMatrix;
#undef Q // undefine shorthand label
}

/**
 * @brief Converts a quaternion to Euler angles in degrees.
 * @param quaternion Quaternion to be converted.
 * @return Euler angles in degrees.
 */
static inline __attribute__((always_inline)) fusion_euler_angles_t fusion_quaternion_to_euler_angles(const fusion_quaternion_t quaternion)
{
#define Q quaternion.element                      // define shorthand label for more readable code
    const float qwqwMinusHalf = Q.w * Q.w - 0.5f; // calculate common terms to avoid repeated operations
    fusion_euler_angles_t eulerAngles;
    eulerAngles.angle.roll = fusion_radians_to_degrees(atan2f(Q.y * Q.z - Q.w * Q.x, qwqwMinusHalf + Q.z * Q.z));
    eulerAngles.angle.pitch = fusion_radians_to_degrees(-1.0f * asinf(2.0f * (Q.x * Q.z + Q.w * Q.y)));
    eulerAngles.angle.yaw = fusion_radians_to_degrees(atan2f(Q.x * Q.y - Q.w * Q.z, qwqwMinusHalf + Q.x * Q.x));
    return eulerAngles;
#undef Q // undefine shorthand label
}

#endif

//------------------------------------------------------------------------------
// End of file
