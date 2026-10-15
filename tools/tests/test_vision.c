#include "img_process.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t image[CONTROL_IMAGE_HEIGHT][CONTROL_IMAGE_WIDTH];
static uint32_t histogram[256];

static void set_lane(int left, int right)
{
    int y, x;
    memset(image, 0, sizeof(image));
    for (y = 0; y < (int)CONTROL_IMAGE_HEIGHT; ++y)
        for (x = left; x <= right; ++x)
            image[y][x] = 255u;
}

static void reset_case(void)
{
    uint16_t y;
    const uint8_t rows[5] = {80u, 85u, 90u, 95u, 100u};
    const float weights[5] = {1.0f, 2.0f, 4.0f, 2.0f, 1.0f};
    vision_reset();
    for (y = 0u; y < CONTROL_IMAGE_HEIGHT; ++y)
        assert(vision_set_reference_width(y, 0u) == 1u);
    assert(vision_set_control_band(rows, weights) == 1u);
}

static void check_coordinates(const VisionResult *result)
{
    uint16_t y;
    for (y = 0u; y < CONTROL_IMAGE_HEIGHT; ++y)
    {
        assert(result->left[y] < CONTROL_IMAGE_WIDTH);
        assert(result->right[y] < CONTROL_IMAGE_WIDTH);
        assert(result->middle[y] < CONTROL_IMAGE_WIDTH);
        assert(result->row_quality[y] <= 2u);
    }
}

static void test_histogram_and_uniform_images(void)
{
    uint8_t *large = (uint8_t *)malloc(70000u);
    VisionResult result;
    uint32_t sum;
    unsigned int gray;
    assert(large != NULL);
    memset(large, 255, 70000u);
    memset(histogram, 0x7f, sizeof(histogram));
    get_hist_gram(large, 280u, 250u, histogram);
    assert(histogram[255] == 70000u);
    assert(histogram[0] == 0u && histogram[200] == 0u);
    assert(get_threshold_otsu(histogram) == 255u);
    memset(large, 0, 70000u);
    get_hist_gram(large, 280u, 250u, histogram);
    assert(histogram[0] == 70000u && histogram[255] == 0u);
    free(large);
    for (gray = 0u; gray < 256u; ++gray)
    {
        reset_case();
        memset(image, (int)gray, sizeof(image));
        get_hist_gram(&image[0][0], CONTROL_IMAGE_HEIGHT, CONTROL_IMAGE_WIDTH, histogram);
        sum = 0u;
        for (unsigned int bin = 0u; bin < 256u; ++bin)
            sum += histogram[bin];
        assert(sum == sizeof(image));
        assert(get_threshold_otsu(histogram) == gray);
        binaryzation_process(&image[0][0], CONTROL_IMAGE_HEIGHT, CONTROL_IMAGE_WIDTH, gray);
        assert(vision_process(&image[0][0], 1u, 20u, &result) == 0u);
        assert(result.track_lost == 1u && result.valid_rows == 0u);
    }
    memset(histogram, 0, sizeof(histogram));
    assert(get_threshold_otsu(histogram) == 0u);
    assert(get_threshold_otsu(NULL) == 0u);
}

static void test_straight_and_weighted_band(void)
{
    VisionResult result;
    uint8_t original[sizeof(image)];
    int y, x;
    reset_case();
    set_lane(60, 128);
    memcpy(original, image, sizeof(image));
    assert(vision_process(&image[0][0], 1u, 20u, &result) == 1u);
    assert(fabsf(result.error_px) < 0.0001f && result.confidence == 1.0f);
    assert(result.valid_rows == CONTROL_IMAGE_HEIGHT && result.reconstructed_rows == 0u);
    assert(memcmp(original, image, sizeof(image)) == 0);
    check_coordinates(&result);
    for (y = 0; y < (int)CONTROL_IMAGE_HEIGHT; ++y)
    {
        int offset = y == 80 ? -10 : (y == 90 ? 10 : 0);
        memset(image[y], 0, CONTROL_IMAGE_WIDTH);
        for (x = 60 + offset; x <= 128 + offset; ++x)
            image[y][x] = 255u;
    }
    assert(vision_process(&image[0][0], 2u, 40u, &result) == 1u);
    assert(fabsf(result.error_px + 3.0f) < 0.0001f);
    memset(image, 0, sizeof(image));
    for (x = 60; x <= 128; ++x)
        image[80][x] = image[90][x] = 255u;
    assert(vision_process(&image[0][0], 3u, 60u, &result) == 0u);
    assert(result.valid_rows == 2u);
}

static void test_mixed_quality_weighting(void)
{
    VisionResult result;
    int x;
    reset_case();
    set_lane(60, 128);
    assert(vision_set_reference_width(90u, 120u) == 1u);
    memset(image[90], 0, CONTROL_IMAGE_WIDTH);
    for (x = 0; x <= 100; ++x)
        image[90][x] = 255u;
    assert(vision_process(&image[0][0], 1u, 20u, &result) == 1u);
    assert(result.row_quality[90] == 1u && result.middle[90] == 40u);
    assert(result.reconstructed_rows == 1u);
    /* 四个实测行权重和为 6，重建行权重由 4 降为 2。
       预期中点为 (6×94+2×40)/8=80.5，偏差为 13.5；
       不降低重建贡献的旧实现会得到偏差 21.6。 */
    assert(fabsf(result.error_px - 13.5f) < 0.0001f);
    assert(fabsf(result.confidence - 0.8f) < 0.0001f);
}

static void test_nearest_run_and_previous_seed(void)
{
    VisionResult result;
    int y, x;
    reset_case();
    set_lane(130, 170);
    assert(vision_process(&image[0][0], 1u, 20u, &result) == 1u);
    assert(result.middle[90] == 150u);
    set_lane(135, 175);
    for (y = 0; y < (int)CONTROL_IMAGE_HEIGHT; ++y)
        for (x = 20; x <= 60; ++x)
            image[y][x] = 255u;
    assert(vision_process(&image[0][0], 2u, 40u, &result) == 1u);
    assert(result.middle[90] == 155u);
    reset_case();
    memset(image, 0, sizeof(image));
    for (y = 0; y < (int)CONTROL_IMAGE_HEIGHT; ++y)
        for (x = 0; x < (int)CONTROL_IMAGE_WIDTH; ++x)
            if ((x >= 100 && x <= 120) || (x >= 150 && x <= 180))
                image[y][x] = 255u;
    assert(vision_process(&image[0][0], 1u, 20u, &result) == 1u);
    assert(result.middle[90] == 110u);
}

static void test_reconstruction_and_frame_identity(void)
{
    VisionResult result, duplicate;
    uint16_t y;
    reset_case();
    set_lane(0, 100);
    assert(vision_process(&image[0][0], 1u, 20u, &result) == 0u);
    assert(result.reconstructed_rows == 0u);
    for (y = 0u; y < CONTROL_IMAGE_HEIGHT; ++y)
        assert(vision_set_reference_width(y, 120u) == 1u);
    vision_reset();
    for (uint32_t frame = 1u; frame <= 3u; ++frame)
    {
        assert(vision_process(&image[0][0], frame, frame * 20u, &result) == 1u);
        assert(result.reconstructed_rows == CONTROL_IMAGE_HEIGHT);
        assert(result.middle[90] == 40u && result.left[90] == 0u);
        assert(result.confidence == 0.5f);
        check_coordinates(&result);
        assert(vision_process(&image[0][0], frame, 9999u, &duplicate) == 1u);
        assert(memcmp(&duplicate, &result, sizeof(result)) == 0);
    }
    assert(vision_process(&image[0][0], 4u, 80u, &result) == 0u);
    assert(vision_process(&image[0][0], 5u, 100u, &result) == 0u);
    assert(vision_process(&image[0][0], 3u, 60u, &result) == 0u);
    set_lane(34, 154);
    assert(vision_process(&image[0][0], 6u, 120u, &result) == 1u);
    set_lane(0, 100);
    assert(vision_process(&image[0][0], 7u, 140u, &result) == 1u);
    vision_reset();
    set_lane(100, 187);
    assert(vision_process(&image[0][0], 1u, 20u, &result) == 1u);
    assert(result.middle[90] == 160u && result.right[90] == 187u);
    memset(image, 255, sizeof(image));
    assert(vision_process(&image[0][0], 2u, 40u, &result) == 0u);
    assert(result.valid_rows == 0u);
}

static void test_configuration_and_legacy_entry(void)
{
    VisionResult result;
    uint8_t left[120], middle[120], right[120];
    uint8_t rows[5] = {80u, 85u, 90u, 95u, 120u};
    float weights[5] = {1.0f, 2.0f, 4.0f, 2.0f, 1.0f};
    reset_case();
    assert(vision_set_reference_width(120u, 50u) == 0u);
    assert(vision_set_reference_width(90u, 188u) == 0u);
    assert(vision_set_reference_width(90u, 2u) == 0u);
    assert(vision_set_control_band(rows, weights) == 0u);
    rows[4] = 100u;
    weights[2] = NAN;
    assert(vision_set_control_band(rows, weights) == 0u);
    weights[2] = 0.0f;
    assert(vision_set_control_band(rows, weights) == 0u);
    weights[2] = 4.0f;
    rows[2] = rows[1];
    assert(vision_set_control_band(rows, weights) == 0u);
    assert(vision_set_control_band(NULL, weights) == 0u);
    rows[2] = 90u;
    for (int index = 0; index < 5; ++index)
        weights[index] = 1.0e36f;
    assert(vision_set_control_band(rows, weights) == 1u);
    assert(vision_process(NULL, 1u, 20u, &result) == 0u);
    assert(vision_process(&image[0][0], 1u, 20u, NULL) == 0u);
    set_lane(60, 128);
    assert(vision_process(&image[0][0], UINT32_MAX, 20u, &result) == 1u);
    assert(isfinite(result.error_px) && fabsf(result.error_px) < 0.0001f);
    assert(vision_process(&image[0][0], 0u, 40u, &result) == 1u);
    auxiliary_process(&image[0][0], 120u, 188u, 0u, left, middle, right);
    assert(left[90] == 60u && middle[90] == 94u && right[90] == 128u);
    memset(image, 0, sizeof(image));
    memset(left, 77, sizeof(left));
    memset(right, 77, sizeof(right));
    auxiliary_process(&image[0][0], 120u, 188u, 0u, left, middle, right);
    assert(left[90] == 0u && right[90] == 187u && middle[90] == 94u);
}

int main(void)
{
    test_histogram_and_uniform_images();
    test_straight_and_weighted_band();
    test_mixed_quality_weighting();
    test_nearest_run_and_previous_seed();
    test_reconstruction_and_frame_identity();
    test_configuration_and_legacy_entry();
    puts("vision: 6 test groups passed (synthetic inputs, real C implementation)");
    return 0;
}
