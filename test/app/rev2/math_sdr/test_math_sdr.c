/*******************************************************************************
 *
 * FILE:
 *      test_math_sdr.c
 *
 * DESCRIPTION:
 *      Unit tests for quaternion vector rotations in math_sdr.
 *
 ******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes
 ------------------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846f

/*------------------------------------------------------------------------------
 Project Includes
 ------------------------------------------------------------------------------*/
#include "math_sdr.h"
#include "sdrtf_pub.h"

/*------------------------------------------------------------------------------
 Test Helpers
 ------------------------------------------------------------------------------*/

static void assert_quat_components
    (
    const char *description,
    QUAT actual,
    QUAT expected
    )
{
TEST_begin_nested_case(description);

TEST_ASSERT_EQ_FLOAT("Quaternion w component", actual.w, expected.w);
TEST_ASSERT_EQ_FLOAT("Quaternion x component", actual.x, expected.x);
TEST_ASSERT_EQ_FLOAT("Quaternion y component", actual.y, expected.y);
TEST_ASSERT_EQ_FLOAT("Quaternion z component", actual.z, expected.z);

TEST_end_nested_case();

} /* assert_quat_components */

/*------------------------------------------------------------------------------
 Tests
 ------------------------------------------------------------------------------*/

void test_world_to_body_identity
    (
    void
    )
{
QUAT attitude =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

QUAT vector_world =
    {
    .w = 0.0f,
    .x = 1.0f,
    .y = 2.0f,
    .z = 3.0f
    };

QUAT actual = quat_rotate_world_to_body(attitude, vector_world);

assert_quat_components
    (
    "Identity world-to-body rotation",
    actual,
    vector_world
    );

} /* test_world_to_body_identity */

void test_body_to_world_identity
    (
    void
    )
{
QUAT attitude =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

QUAT vector_body =
    {
    .w = 0.0f,
    .x = -2.0f,
    .y = 4.0f,
    .z = GRAVITY
    };

QUAT actual = quat_rotate_body_to_world(attitude, vector_body);

assert_quat_components
    (
    "Identity body-to-world rotation",
    actual,
    vector_body
    );

} /* test_body_to_world_identity */

void test_body_world_round_trip
    (
    void
    )
{
const float half_angle = 0.25f * TEST_PI;

QUAT attitude =
    {
    .w = cosf(half_angle),
    .x = 0.0f,
    .y = sinf(half_angle),
    .z = 0.0f
    };

QUAT vector_body =
    {
    .w = 0.0f,
    .x = 1.0f,
    .y = -2.0f,
    .z = 3.0f
    };

QUAT vector_world = quat_rotate_body_to_world
    (
    attitude,
    vector_body
    );

QUAT recovered_body = quat_rotate_world_to_body
    (
    attitude,
    vector_world
    );

assert_quat_components
    (
    "Body-to-world-to-body round trip",
    recovered_body,
    vector_body
    );

} /* test_body_world_round_trip */

void test_positive_90_degree_yaw
    (
    void
    )
{
const float half_angle = 0.25f * TEST_PI;

QUAT attitude =
    {
    .w = cosf(half_angle),
    .x = 0.0f,
    .y = 0.0f,
    .z = sinf(half_angle)
    };

QUAT vector_body =
    {
    .w = 0.0f,
    .x = 1.0f,
    .y = 0.0f,
    .z = 0.0f
    };

QUAT expected_world =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 1.0f,
    .z = 0.0f
    };

QUAT actual = quat_rotate_body_to_world(attitude, vector_body);

assert_quat_components
    (
    "Positive 90 degree yaw rotates body +X to world +Y",
    actual,
    expected_world
    );

} /* test_positive_90_degree_yaw */

void test_stationary_identity_gravity
    (
    void
    )
{
QUAT attitude =
    {
    .w = 1.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

QUAT gravity_world =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = GRAVITY
    };

QUAT measured_accel_body =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = GRAVITY
    };

QUAT gravity_body = quat_rotate_world_to_body
    (
    attitude,
    gravity_world
    );

QUAT linear_accel_body =
    {
    .w = 0.0f,
    .x = measured_accel_body.x - gravity_body.x,
    .y = measured_accel_body.y - gravity_body.y,
    .z = measured_accel_body.z - gravity_body.z
    };

QUAT expected =
    {
    .w = 0.0f,
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f
    };

assert_quat_components
    (
    "Stationary identity attitude removes gravity",
    linear_accel_body,
    expected
    );

} /* test_stationary_identity_gravity */

/*------------------------------------------------------------------------------
 Main
 ------------------------------------------------------------------------------*/

int main
    (
    void
    )
{
unit_test tests[] =
    {
    { "world_to_body_identity", test_world_to_body_identity },
    { "body_to_world_identity", test_body_to_world_identity },
    { "body_world_round_trip", test_body_world_round_trip },
    { "positive_90_degree_yaw", test_positive_90_degree_yaw },
    { "stationary_identity_gravity", test_stationary_identity_gravity }
    };

TEST_INITIALIZE_TEST("math_sdr.c", tests);

} /* main */

/*******************************************************************************
 * END OF FILE
 ******************************************************************************/
