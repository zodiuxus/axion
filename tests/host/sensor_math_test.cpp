/**
 * sensor_math_test.cpp - host tests for the accelerometer offset rescaling.
 *
 * The collision path depends on the accel being read in the range the offsets
 * were scaled for; this arithmetic is the part that can be proven off-target
 *.
 */
#include "test_util.h"
#include "sensor_math.h"

namespace {

void test_identity_at_2g()
{
    CHECK(accel_offset_for_fs(0, 0) == 0);
    CHECK(accel_offset_for_fs(100, 0) == 100);
    CHECK(accel_offset_for_fs(-100, 0) == -100);
    CHECK(accel_offset_for_fs(16383, 0) == 16383);
}

void test_halves_at_4g()
{
    /* AFS_SEL 1 = +-4 g = 8192 LSB/g: the same physical correction is half
     * the raw count. */
    CHECK(accel_offset_for_fs(100, 1) == 50);
    CHECK(accel_offset_for_fs(-100, 1) == -50);
    CHECK(accel_offset_for_fs(16383, 1) == 8191);
}

void test_quarters_at_8g()
{
    CHECK(accel_offset_for_fs(100, 2) == 25);
    CHECK(accel_offset_for_fs(-100, 2) == -25);
}

void test_eighths_at_16g()
{
    CHECK(accel_offset_for_fs(100, 3) == 12);   /* 12.5 truncated */
    CHECK(accel_offset_for_fs(-100, 3) == -12);
}

void test_truncates_toward_zero()
{
    CHECK(accel_offset_for_fs(-1, 1) == 0);
    CHECK(accel_offset_for_fs(-101, 1) == -50);
}

void test_afs_sel_is_masked()
{
    /* AFS_SEL is 2 bits wide; anything above must not shift past the type. */
    CHECK(accel_offset_for_fs(100, 5) == 50);   /* 5 & 3 == 1 */
}

}  // namespace

void run_sensor_math_tests()
{
    test_identity_at_2g();
    test_halves_at_4g();
    test_quarters_at_8g();
    test_eighths_at_16g();
    test_truncates_toward_zero();
    test_afs_sel_is_masked();
}
