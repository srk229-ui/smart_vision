/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */

#include "img_process.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

// 连续白像素至少有3个才作为赛道候选；这是含端点的像素数，不是标定宽度。
#define VISION_MIN_WHITE_RUN (3)
// 同一连续单边区间最多允许3个新帧重建；不是PIT次数，也不是固定毫秒时长。
#define VISION_RECONSTRUCTION_LIMIT (3u)
// 五个控制行至少有3行提供实测或重建中点，还须另行满足置信度门槛。
#define VISION_MIN_CONTROL_ROWS (3u)

// 每个图像行的人工标定宽度，单位像素，定义为右边界减左边界；0表示未标定。
static uint16_t reference_width[CONTROL_IMAGE_HEIGHT];
// 控制使用的实际图像行号，由近端扫描结果中选取；默认集中在图像下部。
static uint8_t control_rows[CONTROL_BAND_COUNT] = {80u, 85u, 90u, 95u, 100u};
// 无量纲相对权重；默认值保持1:2:4:2:1，设置接口会存成归一化比例。
static float control_weights[CONTROL_BAND_COUNT] = {1.0f, 2.0f, 4.0f, 2.0f, 1.0f};
// 上一整体有效帧的加权中点，单位像素；新帧从图像底部以此寻找最近白段。
static int previous_seed = (int)CONTROL_IMAGE_WIDTH / 2;
// 连续出现已标定单边候选的新帧数，达到3后保持3，防止长期依赖虚拟边界。
static uint8_t reconstructed_frames;
// 是否已有可比较帧号的缓存；上电/复位为0，避免把首帧与零编号缓存比较。
static uint8_t previous_frame_present;
// 最近一次处理的新帧结果，包含失效帧；用于同帧返回和帧号先后判断。
static VisionResult previous_result;

// 同步约束：上述状态只在主循环串行访问；本模块不加中断锁。
// 输入图像须由调用者先获得稳定快照；停止态设置标定/控制带也由调用者保证。

// 函数简介：找出距给定种子最近的连续白段，返回包含端点的左右白像素位置。
// 参数说明：row指向一行0/255像素，columns为行宽，seed为上一有效中点。
// 返回参数：找到至少3个白像素的候选为1，否则为0；失败时不提供新边界。
// 备注信息：距离计算到白段本身；触及图像边框是否缺边，由上层判断。
static uint8_t find_white_run(const uint8_t *row, int columns, int seed,
                             int *left, int *right)
{
    // 最大可能距离小于columns+1，保证第一条合格白段能够替换初始值。
    int column = 0, best_distance = columns + 1;
    uint8_t found = 0u;
    while (column < columns)
    {
        int start, end, distance;
        // 先检查列号再读取像素，短路判断保证不会访问行尾之外的内存。
        while (column < columns && row[column] != 255u)
            ++column;
        start = column; // 第一个白像素；若已到行尾，后面的长度检查会拒绝空段。
        while (column < columns && row[column] == 255u)
            ++column;
        end = column - 1; // 最后一个白像素，因此白段长度需要加1。
        // 丢弃不足3像素的小白点；也处理扫到行尾但没有白段的空候选。
        if (end - start + 1 < VISION_MIN_WHITE_RUN)
            continue;
        // 种子在白段内时距离为0；在两侧时到最近端点的距离为非负像素数。
        // 这里不是比较白段中点距离，同距时保留从左向右首先发现的白段。
        distance = seed < start ? start - seed : (seed > end ? seed - end : 0);
        if (distance < best_distance)
        {
            *left = start;
            *right = end;
            best_distance = distance;
            found = 1u;
        }
    }
    return found;
}

// 函数简介：把可能位于画面外的有符号虚拟边界转换成可显示的列号。
// 参数说明：column单位为像素，可小于0或大于187。
// 返回参数：0～187的显示坐标。
// 备注信息：只限幅边界输出；控制中点必须先由未限幅边界计算。
static uint8_t clip_column(int column)
{
    if (column < 0)
        return 0u;
    if (column >= (int)CONTROL_IMAGE_WIDTH)
        return (uint8_t)(CONTROL_IMAGE_WIDTH - 1u);
    return (uint8_t)column;
}

// 函数简介：开始新试验时清除视觉跨帧状态，不改变已审核的空间配置。
// 参数说明：无。
// 返回参数：无。
// 备注信息：主循环串行调用；不会切换控制状态或触发电机输出。
void vision_reset(void)
{
    // 只清种子、连续重建计数和缓存；人工标定宽度、五行前瞻及权重均保留。
    previous_seed = (int)CONTROL_IMAGE_WIDTH / 2;
    reconstructed_frames = 0u;
    previous_frame_present = 0u;
    memset(&previous_result, 0, sizeof(previous_result));
}

// 函数简介：设置一行人工居中直道标定得到的宽度，零值可清除标定。
// 参数说明：row为图像行号；width为左右白段端点之差，单位像素。
// 返回参数：写入成功为1，参数不合法为0且不写入。
// 备注信息：主循环在停止态调用，接口本身不读取或强制控制状态。
uint8_t vision_set_reference_width(uint16_t row, uint16_t width)
{
    // 行号必须在图像内；非零宽度须为3～187，0单独允许以清除该行标定。
    // 扫描的3个连续白像素端点差为2，不能把候选长度直接当作可标定宽度。
    if (row >= CONTROL_IMAGE_HEIGHT ||
        (width != 0u && (width < VISION_MIN_WHITE_RUN || width >= CONTROL_IMAGE_WIDTH)))
        return 0u;
    reference_width[row] = width;
    return 1u;
}

// 函数简介     只读查询人工直道宽度。
// 参数说明     row为0到119图像行号。
// 返回参数     像素宽度；未标定或越界返回0。
// 备注信息     不改变原寻线历史，不写入估计宽度。
uint16_t vision_get_reference_width(uint16_t row)
{
    return row < CONTROL_IMAGE_HEIGHT ? reference_width[row] : 0u;
}

// 函数简介：先完整校验并归一化，再提交五行控制带配置。
// 参数说明：rows来自停止态参数配置；weights为各行的有限正相对权重。
// 返回参数：成功为1；任何一项失败为0，原配置保持。
// 备注信息：两次memcpy由主循环串行完成，并非对其他并发访问者的原子操作。
uint8_t vision_set_control_band(const uint8_t rows[CONTROL_BAND_COUNT],
                                const float weights[CONTROL_BAND_COUNT])
{
    uint8_t index;
    float total = 0.0f; // 五个原始权重之和，供比例归一化使用。
    float normalized[CONTROL_BAND_COUNT]; // 校验完成前只写局部数组，不提前发布配置。
    if (rows == NULL || weights == NULL)
        return 0u;
    for (index = 0u; index < CONTROL_BAND_COUNT; ++index)
    {
        // 每行必须在0～119；权重须有限且大于0；除首行外行号须严格递增。
        // index非零的判断先于读取前一项，避免首行发生数组下标下溢。
        if (rows[index] >= CONTROL_IMAGE_HEIGHT || !isfinite(weights[index]) ||
            weights[index] <= 0.0f || (index != 0u && rows[index] <= rows[index - 1u]))
            return 0u;
        total += weights[index];
    }
    // 各项有限仍可能在求和时溢出；非有限总和不能作为分母。
    if (!isfinite(total))
        return 0u;
    // 先归一化再乘像素坐标，避免有限但很大的原权重乘坐标后溢出。
    for (index = 0u; index < CONTROL_BAND_COUNT; ++index)
    {
        normalized[index] = weights[index] / total;
        // 检查浮点异常与下溢为0，保证每个控制行仍有可用的正权重。
        if (!isfinite(normalized[index]) || normalized[index] <= 0.0f)
            return 0u;
    }
    // 所有检查通过后才提交两组配置；调用者须保证此时没有并发读写。
    memcpy(control_rows, rows, sizeof(control_rows));
    memcpy(control_weights, normalized, sizeof(control_weights));
    return 1u;
}

// 函数简介：将一幅188×120二值快照转换为可供转向控制使用的视觉结果。
// 参数说明：frame_id来自相机计数；capture_time_ms来自采集回调的时间标签。
// 返回参数：整体有效为1，否则为0；逐行质量与失效标志写入result。
// 备注信息：不读时钟、不修改输入；当前设备时间是10ms量化的逻辑时间。
//           本函数不生成Otsu阈值，调用者须在每次返回后填写result->threshold。
uint8_t vision_process(const uint8_t *binary_after_otsu, uint32_t frame_id,
                       uint32_t capture_time_ms, VisionResult *result)
{
    int row, seed = previous_seed; // 新帧继承上一整体有效帧的中点，行内继续向上更新。
    uint8_t single_edge_present = 0u, contributing_rows = 0u, index;
    float weighted_middle = 0.0f, available_weight = 0.0f; // 中点加权和与有效权重和。
    float quality_weight = 0.0f, total_weight = 0.0f; // 质量折算权重与完整控制带权重。

    // 1.先处理输出指针和重复帧；同一帧不重新扫描，也不能延长保护所见的寿命。
    if (result == NULL)
        return 0u;
    if (previous_frame_present != 0u && frame_id == previous_result.frame_id)
    {
        // 返回原缓存及原采集标签，不采用本次时间参数、不消耗新的重建帧许可。
        // 重复帧可直接返回缓存，不依赖本次图像指针；阈值仍由调用者补写。
        *result = previous_result;
        return result->valid;
    }

    // 2.先建立明确的失效结果；零质量的默认边界/中点仅供绘图，不能参与控制。
    memset(result, 0, sizeof(*result));
    result->frame_id = frame_id;
    result->capture_time_ms = capture_time_ms; // 保留采集标签，不换成视觉处理完成时间。
    result->track_lost = 1u; // 只有后面的控制带质量通过，才清除此标志。
    for (row = 0; row < (int)CONTROL_IMAGE_HEIGHT; ++row)
    {
        // left保持清零后的0；middle=94、right=187仅表示显示占位。
        result->middle[row] = (uint8_t)(CONTROL_IMAGE_WIDTH / 2u);
        result->right[row] = (uint8_t)(CONTROL_IMAGE_WIDTH - 1u);
    }
    // 新帧须有图像且帧号在缓存之后；先作无符号差再转有符号支持最大值回绕至0。
    // 此比较要求相邻待比较帧号间隔小于2^31；拒绝旧帧时不更新跨帧缓存或种子。
    if (binary_after_otsu == NULL || (previous_frame_present != 0u &&
        (int32_t)(frame_id - previous_result.frame_id) <= 0))
        return 0u;

    // 3.从图像底部向上扫描；有符号行号可在0之后减至-1并正常结束。
    for (row = (int)CONTROL_IMAGE_HEIGHT - 1; row >= 0; --row)
    {
        // 边界使用有符号临时量，允许人工宽度重建出的虚拟边界位于画面之外。
        int left = 0, right = (int)CONTROL_IMAGE_WIDTH - 1, middle;
        uint8_t quality = 0u; // 0无证据、1单边重建、2双边实测。
        // 一维快照按每行188字节定位；没有合格白段便保留本行零质量占位。
        if (find_white_run(binary_after_otsu + (size_t)row * CONTROL_IMAGE_WIDTH,
                           (int)CONTROL_IMAGE_WIDTH, seed, &left, &right) == 0u)
            continue;
        // 白段左右端点都未触及边框，才表示两侧均能在本图像内实测。
        if (left > 0 && right < (int)CONTROL_IMAGE_WIDTH - 1)
        {
            int measured_width = right - left; // 两白段端点的坐标差，单位像素。
            // 未标定时接受双实测；已有标定时宽度须在参考值的0.5～1.5倍内。
            // 将两侧乘2作整数比较，排除明显小白斑/错误白段且避免浮点除法。
            if (reference_width[row] == 0u ||
                (measured_width * 2 >= reference_width[row] &&
                 measured_width * 2 <= reference_width[row] * 3))
                quality = 2u;
        }
        // 至少一侧不触及边框且本行已标定，才有根据重建缺失的另一侧。
        // 全白行两端都触边，未标定行宽度为0；两种情况均不能伪造有效中线。
        else if ((left > 0 || right < (int)CONTROL_IMAGE_WIDTH - 1) &&
                 reference_width[row] != 0u)
        {
            // 先记录整帧存在单边候选，达到上限仍保持此标志，避免第4帧重置许可。
            single_edge_present = 1u;
            if (reconstructed_frames < VISION_RECONSTRUCTION_LIMIT)
            {
                // 参考宽度与当前实测边界同为像素；先构造未限幅的虚拟另一边。
                if (left == 0)
                    left = right - (int)reference_width[row]; // 左侧缺失：右实测边减宽度。
                else
                    right = left + (int)reference_width[row]; // 右侧缺失：左实测边加宽度。
                quality = 1u;
            }
        }
        // 先以未限幅边界求整数中点；C整数除法向0截断，随后检查该整数坐标。
        middle = (left + right) / 2;
        // 无可信边界或中点出画面都不发布本行；质量为0的显示占位保持不变。
        if (quality == 0u || middle < 0 || middle >= (int)CONTROL_IMAGE_WIDTH)
            continue;
        result->left[row] = clip_column(left); // 仅显示边界限幅，不反向改变控制中点。
        result->right[row] = clip_column(right);
        result->middle[row] = (uint8_t)middle;
        result->row_quality[row] = quality; // 保存来源，供五行控制带按质量降权。
        ++result->valid_rows; // 整张图像中实测行与重建行的总数，非仅五个控制行。
        if (quality == 1u)
            ++result->reconstructed_rows; // 重建行数是有效行数的子集。
        seed = middle; // 本行有效中点成为下一行种子，沿连贯赛道向上跟随。
    }

    // 4.按整张图像的新帧更新重建期限，一帧即使多行重建也只增加一次。
    if (single_edge_present != 0u)
    {
        if (reconstructed_frames < VISION_RECONSTRUCTION_LIMIT)
            ++reconstructed_frames; // 第1～3个单边候选帧允许重建，之后饱和保持3。
    }
    else
        // 没有任何已标定单边候选便清零；双边恢复或全黑丢失都可能到达这里。
        reconstructed_frames = 0u;

    // 5.只汇总五个控制行：实测质量系数1，重建0.5，无效0。
    for (index = 0u; index < CONTROL_BAND_COUNT; ++index)
    {
        uint8_t y = control_rows[index]; // 设置接口已保证此行号小于120。
        float weight = control_weights[index]; // 配置中的相对权重，无量纲。
        float effective_weight;
        total_weight += weight; // 分母保留全部配置权重，无效行也占应有证据份额。
        if (result->row_quality[y] == 0u)
            continue;
        ++contributing_rows; // 实测/重建均计一行，后面再用质量系数限制可信度。
        // 中点和置信度都用weight×质量系数，混合行时重建贡献减半。
        effective_weight = weight * (result->row_quality[y] == 2u ? 1.0f : 0.5f);
        available_weight += effective_weight; // 有效中点重新归一化的分母，避免无效行拉向0。
        weighted_middle += effective_weight * result->middle[y]; // 加权像素坐标之和。
        quality_weight += effective_weight; // 置信度分子，与完整五行证据相比。
    }
    // 置信度=sum(weight×质量)/sum(weight)，范围0～1；重建全齐时仍只有0.5。
    result->confidence = quality_weight / total_weight;

    // 6.至少3个控制行、有正的归一化分母且置信度≥0.35，才发布可控制的中点。
    if (contributing_rows >= VISION_MIN_CONTROL_ROWS && available_weight > 0.0f &&
        result->confidence >= 0.35f)
    {
        // 偏差=图像中心94-质量加权中点，单位像素；赛道在图像左侧时为正。
        // 它是图像偏差，不是已标定的厘米或舵机角；转向方向由控制层转换。
        result->error_px = (float)(CONTROL_IMAGE_WIDTH / 2u) - weighted_middle / available_weight;
        result->valid = 1u;
        result->track_lost = 0u;
        // 只有整体有效帧更新跨帧种子；失败帧的零误差不能被当作车辆已居中。
        previous_seed = (int)(weighted_middle / available_weight);
    }
    // 缓存本次新帧，包括失效结果，以便识别后续重复帧；本函数不直接驱动电机。
    previous_result = *result;
    previous_frame_present = 1u;
    return result->valid;
}

// 函数简介：使用Otsu阈值将灰度缓冲区原地改写为0/255二值图。
// 参数说明：尺寸由调用者提供，阈值为灰度级；缓冲区须至少有rows×cols字节。
// 返回参数：无。
// 备注信息：灰度值等于阈值归为黑色；若需保留灰度图须在调用前保存。
void binaryzation_process(unsigned char *_img, const unsigned short _rows,
                          const unsigned short _cols, const unsigned int _threshold_value)
{
    // 乘法前转为32位无符号数，避免整数提升后的有符号乘积溢出。
    uint32_t index, size = (uint32_t)_rows * _cols;
    if (_img == NULL)
        return;
    for (index = 0u; index < size; ++index)
        // 严格大于阈值才为255；等于阈值为0，与白段扫描只接受255的约定一致。
        _img[index] = _img[index] > _threshold_value ? 255u : 0u;
}

// 函数简介：完整清零后统计8位图像的256级灰度直方图。
// 参数说明：图像缓冲区及尺寸；输出须提供256个uint32_t计数空间。
// 返回参数：无。
// 备注信息：图像为空时仍清零有效输出；计数类型可表示超过32767的像素数。
void get_hist_gram(unsigned char *_img, const unsigned short _rows,
                   const unsigned short _cols, uint32_t *_hist_gram)
{
    uint32_t index, size = (uint32_t)_rows * _cols;
    if (_hist_gram == NULL)
        return;
    // 清除256个32位计数的全部1024字节，不能沿用旧16位直方图的清零长度。
    memset(_hist_gram, 0, sizeof(uint32_t) * 256u);
    if (_img == NULL)
        return;
    for (index = 0u; index < size; ++index)
        // 8位像素本身是0～255的合法灰度下标，每读取一个像素增加对应灰度计数。
        ++_hist_gram[_img[index]];
}

// 函数简介：遍历可能分割点，以最大类间差异选择Otsu灰度阈值。
// 参数说明：256级32位灰度计数，灰度值为0～255。
// 返回参数：0～255阈值；空图返回0，纯色图返回该颜色灰度。
// 备注信息：无非空两类时不除零；配合严格大于阈值的二值化使纯色图全黑。
unsigned char get_threshold_otsu(const uint32_t *_hist_gram)
{
    uint16_t gray; // 需遍历至256结束，不能用到255后回绕的8位循环变量。
    // count为像素总数，integral为灰度×像素总和；back前缀是低灰度类的累加量。
    // 64位累加与乘法避免多个32位桶相加及灰度乘计数时溢出。
    uint64_t count = 0u, integral = 0u, back_count = 0u, back_integral = 0u;
    double best_variance = -1.0; // 分数非负，-1保证第一处有效分割能被采用。
    uint8_t threshold = 0u;
    if (_hist_gram == NULL)
        return 0u;

    // 1.先求总量；最后一个非空灰度作为没有可用分割时的后备阈值。
    for (gray = 0u; gray < 256u; ++gray)
    {
        count += _hist_gram[gray];
        integral += (uint64_t)_hist_gram[gray] * gray; // 提升后再乘，单位灰度×像素。
        if (_hist_gram[gray] != 0u)
            threshold = (uint8_t)gray;
    }
    if (count == 0u)
        return 0u;

    // 2.候选阈值0～254：低灰度类包含≤gray，高灰度类包含>gray。
    // 255不可能留下非空高灰度类，因此无需作为分割点参与评分。
    for (gray = 0u; gray < 255u; ++gray)
    {
        uint64_t fore_count; // 高灰度类像素数，由总数减去已累加的低灰度类。
        double delta, variance;
        back_count += _hist_gram[gray];
        back_integral += (uint64_t)_hist_gram[gray] * gray;
        if (back_count == 0u)
            continue; // 低类为空，不能求低类均值，继续寻找首个非空桶。
        fore_count = count - back_count;
        if (fore_count == 0u)
            break; // 高类已空，之后更大阈值也无法形成两类，结束扫描。

        // 两类平均灰度之差；使用double除法保留小数，不进行整数截断。
        delta = (double)back_integral / (double)back_count -
                (double)(integral - back_integral) / (double)fore_count;
        // 分数=n低×n高×(均值差)^2；省略各候选共同的总像素数平方分母，
        // 不改变最大分数对应的阈值；先转double再相乘，避免整数乘积溢出。
        variance = (double)back_count * (double)fore_count * delta * delta;
        if (variance > best_variance)
        {
            best_variance = variance;
            threshold = (uint8_t)gray; // 分数相同不更新，保留先遇到的较低阈值。
        }
    }
    return threshold; // 纯色输入无有效分割，返回前面记录的唯一非空灰度。
}

// 函数简介：保留旧调用接口，对二值图逐行输出双实测边线与中点。
// 参数说明：尺寸与输出数组由旧调用者提供；_threshold_val只保留签名兼容。
// 返回参数：无。
// 备注信息：没有质量标志或单边重建，不用于当前驱动车辆的控制链。
void auxiliary_process(uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                        unsigned char _threshold_val, uint8_t *_left_line,
                        uint8_t *_mid_line, uint8_t *_right_line)
{
    int row, seed = _src_cols / 2; // 此兼容入口每次从画面中心起始，不沿用主视觉帧缓存。
    // 接收的已经是0/255二值图，灰度阈值不再参与判白，避免阈值0把黑色误当白色。
    (void)_threshold_val;
    // 所有输入/输出都须非空且尺寸非零，再计算末行、末列以防下溢或无效写入。
    if (_src_pixel_mat == NULL || _left_line == NULL || _mid_line == NULL ||
        _right_line == NULL || _src_rows == 0u || _src_cols == 0u)
        return;
    for (row = (int)_src_rows - 1; row >= 0; --row)
    {
        int left = 0, right = _src_cols - 1;
        // 逐行重置显示占位，旧帧边线不会残留；兼容结果不能单凭中点占位判断有效。
        _left_line[row] = 0u;
        _right_line[row] = (uint8_t)(_src_cols - 1u);
        _mid_line[row] = (uint8_t)(_src_cols / 2u);
        // 白段搜索成功且两边都在画面内部，才输出本行实测；单边/全白均不采用。
        if (find_white_run(_src_pixel_mat + (size_t)row * _src_cols, _src_cols,
                           seed, &left, &right) != 0u && left > 0 && right < _src_cols - 1)
        {
            _left_line[row] = (uint8_t)left;
            _right_line[row] = (uint8_t)right;
            _mid_line[row] = (uint8_t)((left + right) / 2);
            seed = _mid_line[row]; // 向上一行传递实测中点，提高相邻行连续性。
        }
    }
}
