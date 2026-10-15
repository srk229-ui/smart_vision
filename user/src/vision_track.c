#include "vision_track.h"
#include "img_process.h"
#ifndef VISION_TRACK_MAP_HEADER
#define VISION_TRACK_MAP_HEADER "vision_track_map.h" // 实机默认估计映射；宿主可选择独立标定表验证，不覆盖实机表。
#endif
#include VISION_TRACK_MAP_HEADER
#include <math.h>
#include <string.h>
#include <stdlib.h> // 连续边线的整数位移检查。

typedef struct
{
    uint16_t nodes[VISION_TRACK_POINTS]; // 鸟瞰一维格索引，每条路径禁止重复格。
    uint16_t count; // 已使用节点数，范围1到80。
    float score; // 累积方向变化惩罚，数值越小越连续。
    float length_m; // 格步长累计米数，斜向步长为0.02乘根号2。
    float dir_row; // 上一步横向格方向的单位分量。
    float dir_col; // 上一步前向格方向的单位分量。
} TrackPath;

uint8_t vision_track_control_rows[CONTROL_BAND_COUNT]; // 保留自适应控制带供停车记录，不改原配置。
volatile VisionTimingLog vision_timing_log; // 主循环唯一写入者；PIT不访问大记录区。
static VisionTimingRecord timing_record; // 正在构建的一帧，只在finish时发布。
static uint32_t timing_started_us; // 复制开始的GPT微秒，用无符号差分计算整帧耗时。
static uint32_t (*read_us)(void); // 可注入宿主时钟，实机安装独立GPT时基。
static uint8_t road_mask[VISION_TRACK_CELLS]; // 当前可见白路，保持为0或1，不在细化时覆盖。
static uint8_t skeleton[VISION_TRACK_CELLS]; // 细化工作区，每轮删除后更新。
static uint8_t delete_mask[VISION_TRACK_CELLS]; // 子步骤同步删除标志，避免扫描顺序改变拓扑。
static uint8_t delete_table[2][256]; // 两个子步骤各256种邻居排列的删除条件。
static TrackPath beams[2][VISION_TRACK_BEAMS]; // 交替保存本层与下一层候选，禁止动态分配。
static TrackPath chosen_path; // 已选路径，搜索结束后用于投影。
static float projected_u[VISION_TRACK_POINTS + 25u]; // 选中路径及有观测支持的近端延伸像素列。
static float projected_v[VISION_TRACK_POINTS + 25u]; // 与projected_u成对的像素行。
static VisionResult cached_result; // 重复帧返回原采集标签，不消耗新的路径恢复。
static uint8_t frame_seen, history_valid; // 是否已有帧和近期成功路径；失败不延长历史资格。
static uint32_t last_frame_id, history_ms; // 帧序号及成功路径的采集逻辑毫秒。
static float history_x, history_y; // 上次路径的远端车体坐标，仅用于短时连续性评分。


// 地面白段只沿近端已选道路向上延伸；不让墙面或另一条白路成为新的扫描起点。
static uint8_t guarded_binary[CONTROL_IMAGE_WIDTH*CONTROL_IMAGE_HEIGHT];
static uint16_t learned_width[CONTROL_IMAGE_HEIGHT]; // 最近双边实测宽度，不能把估计宽度保存成实测。
static uint32_t width_time[CONTROL_IMAGE_HEIGHT]; // 宽度对应采集时间，单边只能短时使用最近实测。
static uint8_t guard_adaptive, guard_single; // 本帧是否改用较远控制带、是否使用当前单边。
static uint8_t guard_kind[CONTROL_IMAGE_HEIGHT]; // 0无道路，1只见右边，2只见左边，3双边；边框不算真实边线。
static float edge_x[2][CONTROL_IMAGE_HEIGHT], edge_y[2][CONTROL_IMAGE_HEIGHT]; // 左/右真实边点的地面X前/Y左，米。
static uint8_t edge_valid[2][CONTROL_IMAGE_HEIGHT]; // 当前帧边点通过映射且在鸟瞰范围内为1。
static float normal_middle[CONTROL_IMAGE_HEIGHT]; // 法向中线重投影到各图像行的亚像素列。
static uint8_t normal_hits[CONTROL_IMAGE_HEIGHT]; // 0无交点，1一致交点，2存在歧义；歧义行不用于控制。

static uint32_t now_us(void);

// 函数简介     对归一化相机点施加Brown畸变，双向变换共享同一组离线配置。
// 参数说明     x/y为归一化坐标；dx/dy接收畸变后坐标，不是像素。
// 返回参数     void
// 备注信息     默认估计系数全零；H模式也要求全零，不能把校正坐标和原图混用。
static void distort_point(float x, float y, float *dx, float *dy)
{
    const float *d=vision_map_distortion;
    float r=x*x+y*y, radial=1.0f+r*(d[0]+r*(d[1]+r*d[4]));
    *dx=x*radial+2.0f*d[2]*x*y+d[3]*(r+2.0f*x*x);
    *dy=y*radial+d[2]*(r+2.0f*y*y)+2.0f*d[3]*x*y;
}

// 函数简介     将真实可见的原图边点变换到地面，不量化到2厘米格。
// 参数说明     u/v为像素列/行；x/y接收米，X向前、Y向左。
// 返回参数     落在当前鸟瞰物理范围为1；地平线、畸变不收敛或范围外为0。
// 备注信息     矩阵由离线生成器求逆；仅有畸变时最多迭代8次，工作量固定有界。
static uint8_t image_ground_point(float u, float v, float *x, float *y)
{
    const float *h=vision_map_image_to_ground, *k=vision_map_intrinsics, *d=vision_map_distortion;
    float scale;
    if(d[0]!=0.0f || d[1]!=0.0f || d[2]!=0.0f || d[3]!=0.0f || d[4]!=0.0f) {
        float xd=(u-k[2])/k[0], yd=(v-k[3])/k[1], xx=xd, yy=yd, dx,dy;
        unsigned i;
        for(i=0u;i<8u;++i) {
            distort_point(xx,yy,&dx,&dy); xx+=xd-dx; yy+=yd-dy;
            if(!isfinite(xx) || !isfinite(yy) || fabsf(xx)>4.0f || fabsf(yy)>4.0f) return 0u;
        }
        distort_point(xx,yy,&dx,&dy);
        if(fabsf(dx-xd)*k[0]>0.05f || fabsf(dy-yd)*k[1]>0.05f) return 0u;
        u=k[0]*xx+k[2]; v=k[1]*yy+k[3];
    }
    scale=h[6]*u+h[7]*v+h[8];
    if(!isfinite(scale) || fabsf(scale)<1.0e-6f) return 0u;
    *x=(h[0]*u+h[1]*v+h[2])/scale; *y=(h[3]*u+h[4]*v+h[5])/scale;
    return (uint8_t)(isfinite(*x) && isfinite(*y) && *x>=0.06f && *x<=1.84f && fabsf(*y)<=1.15f);
}

// 函数简介     将地面中线反投影到原图，保持现有像素误差和PD接口。
// 参数说明     x/y单位米；u/v返回带原图畸变的浮点像素。
// 返回参数     投影在地面ROI及188×120图像内为1，其他为0。
// 备注信息     不刷新采集时间，不把画面外中线裁剪为边界上的有效点。
static uint8_t ground_image_point(float x, float y, float *u, float *v)
{
    const float *h=vision_map_ground_to_image, *k=vision_map_intrinsics;
    float scale=h[6]*x+h[7]*y+h[8], dx,dy;
    if(!isfinite(scale) || fabsf(scale)<1.0e-6f || x<0.06f || x>1.84f || fabsf(y)>1.15f) return 0u;
    *u=(h[0]*x+h[1]*y+h[2])/scale; *v=(h[3]*x+h[4]*y+h[5])/scale;
    distort_point((*u-k[2])/k[0],(*v-k[3])/k[1],&dx,&dy);
    *u=k[0]*dx+k[2]; *v=k[1]*dy+k[3];
    return (uint8_t)(isfinite(*u) && isfinite(*v) && *u>=0.0f && *u<=187.0f &&
                     *v>=(float)VISION_GROUND_TOP_ROW && *v<=119.0f);
}

// 函数简介     沿当前可见边线两侧取短弧长窗口，用多点跨度估计从近到远的切向。
// 参数说明     side为0左边/1右边；row为原图行；tx/ty接收地面单位切向。
// 返回参数     至少3个连续点、跨度足够且无折返为1，否则0。
// 备注信息     边点消失即停止，禁止跨越图像边框、背景或缺失段拼接法向。
static uint8_t edge_tangent(unsigned side, int row, float *tx, float *ty)
{
    int ends[2]={row,row}, direction;
    unsigned points=1u;
    float arc=0.0f, span;
    for(direction=-1;direction<=1;direction+=2) {
        int current=row, index=direction<0 ? 0 : 1;
        float length=0.0f;
        while(length<VISION_NORMAL_WINDOW_M) {
            int next=current+direction;
            float step;
            if(next<(int)VISION_GROUND_TOP_ROW || next>=120 || !edge_valid[side][next]) break;
            step=hypotf(edge_x[side][next]-edge_x[side][current],edge_y[side][next]-edge_y[side][current]);
            if(step>VISION_NORMAL_MAX_STEP_M) break;
            length+=step; current=next; ++points;
        }
        ends[index]=current; arc+=length;
    }
    // 图像行从大到小为当前边线的近到远次序；不以Y符号猜测左右弯。
    *tx=edge_x[side][ends[0]]-edge_x[side][ends[1]];
    *ty=edge_y[side][ends[0]]-edge_y[side][ends[1]];
    span=hypotf(*tx,*ty);
    if(points<3u || span<VISION_NORMAL_MIN_SPAN_M || arc>1.25f*span) return 0u;
    *tx/=span; *ty/=span;
    return 1u;
}

// 函数简介     将一小段法向中线与图像行求交，只接受当前单边白路内的交点。
// 参数说明     kind为1右边可见/2左边可见；u/v为前后两个恢复点；result提供真实白段边界。
// 返回参数     void
// 备注信息     多次求交差异超过3像素标为歧义；不从多个分支平均出一条假中线。
static void normal_intersections(unsigned kind, float u0, float v0, float u1, float v1, const VisionResult *result)
{
    unsigned row;
    if(fabsf(v1-v0)<0.001f) return;
    for(row=VISION_GROUND_TOP_ROW;row<CONTROL_IMAGE_HEIGHT;++row) {
        float u;
        int col;
        if(guard_kind[row]!=kind || ((float)row-v0)*((float)row-v1)>0.0f) continue;
        u=u0+(u1-u0)*(((float)row-v0)/(v1-v0)); col=(int)floorf(u+0.5f);
        if(col<result->left[row] || col>result->right[row] || col<0 || col>=188 || !guarded_binary[row*188u+(unsigned)col]) continue;
        if(normal_hits[row]==0u) { normal_middle[row]=u; normal_hits[row]=1u; }
        else if(fabsf(normal_middle[row]-u)>3.0f) normal_hits[row]=2u;
    }
}

// 函数简介     将可见边线映射到地面，沿道路内法向偏移半个配置路宽后重投影。
// 参数说明     result来自近端连续道路筛选，双边行保持原实测中点。
// 返回参数     void；仅把有当前白路支持的单边行标记质量1。
// 备注信息     不使用像素参考宽度，不需要上一帧双边标定，也不使用裁剪边界求切向。
static void recover_normal(VisionResult *result)
{
    unsigned side,row;
    memset(edge_valid,0,sizeof(edge_valid)); memset(normal_hits,0,sizeof(normal_hits));
    for(row=VISION_GROUND_TOP_ROW;row<CONTROL_IMAGE_HEIGHT;++row) {
        if(!guard_kind[row]) continue;
        for(side=0u;side<2u;++side) {
            unsigned col=side ? result->right[row] : result->left[row];
            if(col==0u || col==187u) continue;
            edge_valid[side][row]=image_ground_point((float)col,(float)row,&edge_x[side][row],&edge_y[side][row]);
        }
    }
    for(side=0u;side<2u;++side) {
        unsigned kind=side ? 1u : 2u;
        float last_x=0.0f,last_y=0.0f,last_u=0.0f,last_v=0.0f,last_tx=0.0f,last_ty=0.0f;
        int have_last=0;
        for(row=CONTROL_IMAGE_HEIGHT;row-- > VISION_GROUND_TOP_ROW;) {
            float tx,ty,x,y,u,v,offset=(side ? 0.5f : -0.5f)*VISION_MAP_ROAD_WIDTH_M;
            if(!edge_valid[side][row] || !edge_tangent(side,(int)row,&tx,&ty)) { have_last=0; continue; }
            // 左法向为(-ty,tx)；左边向右减半宽，右边向左加半宽，X/Y同时改变。
            x=edge_x[side][row]-offset*ty; y=edge_y[side][row]+offset*tx;
            if(!ground_image_point(x,y,&u,&v)) { have_last=0; continue; }
            if(have_last && hypotf(x-last_x,y-last_y)<=VISION_NORMAL_MAX_STEP_M && tx*last_tx+ty*last_ty>0.5f) {
                // 内偏移在极小半径处可能折返；拒绝方向反转，不能连接成穿越赛道的中线。
                if((x-last_x)*tx+(y-last_y)*ty>0.0f) normal_intersections(kind,last_u,last_v,u,v,result);
            }
            last_x=x; last_y=y; last_u=u; last_v=v; last_tx=tx; last_ty=ty; have_last=1;
        }
    }
    for(row=VISION_GROUND_TOP_ROW;row<CONTROL_IMAGE_HEIGHT;++row) {
        if(normal_hits[row]!=1u || result->row_quality[row]!=0u) continue;
        result->middle[row]=(uint8_t)floorf(normal_middle[row]+0.5f); result->row_quality[row]=1u;
        ++result->valid_rows; ++result->reconstructed_rows; guard_single=1u;
    }
}

// 函数简介     汇总连续道路中的控制行，固定带不足时选择邻近的连续有效五行。
// 参数说明     params是原控制带；result只含当前帧通过道路约束的行。
// 返回参数     当前证据满足门槛为1；不刷新帧号或采集时间。
static uint8_t summarize_guard(const ControlParams *params, VisionResult *result)
{
    unsigned i, usable=0u, selected[CONTROL_BAND_COUNT];
    float available=0.0f, total=0.0f, weighted=0.0f;
    guard_adaptive=0u;
    for(i=0u;i<CONTROL_BAND_COUNT;++i) {
        selected[i]=params->look_rows[i];
        usable+=result->row_quality[selected[i]]!=0u;
        total+=params->look_weights[i];
        if(result->row_quality[selected[i]])
            available+=params->look_weights[i]*(result->row_quality[selected[i]]==2u ? 1.0f : 0.5f);
    }
    if(usable<3u || total<=0.0f || available/total<0.35f) {
        int distance, direction;
        // 法向偏移同时改变X，重投影可能前移或后移；在原带附近先远后近逐级寻找。
        // 候选中心55到113确保五个采样行均在ROI中，且必须已有当前道路有效证据。
        for(distance=0;distance<120 && !guard_adaptive;++distance) {
            for(direction=-1;direction<=1;direction+=2) {
                int center=(int)params->look_rows[2]+direction*distance;
                if(center<55 || center>113 || (distance==0 && direction==1)) continue;
                for(i=0u;i<CONTROL_BAND_COUNT;++i) {
                    unsigned row=(unsigned)(center+((int)i-2)*3);
                    if(result->row_quality[row]==0u) break;
                    selected[i]=row;
                }
                if(i==CONTROL_BAND_COUNT) { guard_adaptive=1u; break; }
            }
        }
        if(!guard_adaptive) return 0u;
    }
    usable=0u; total=0.0f; available=0.0f;
    for(i=0u;i<CONTROL_BAND_COUNT;++i) {
        unsigned row=selected[i];
        float w=params->look_weights[i];
        total+=w;
        if(result->row_quality[row]) {
            float q=result->row_quality[row]==2u ? 1.0f : 0.5f;
            available+=w*q; weighted+=w*q*result->middle[row]; ++usable;
        }
    }
    if(usable<3u || total<=0.0f || available/total<0.35f) return 0u;
    result->confidence=available/total;
    if(guard_adaptive && result->confidence>0.6f) result->confidence=0.6f;
    for(i=0u;i<CONTROL_BAND_COUNT;++i) vision_track_control_rows[i]=(uint8_t)selected[i];
    result->error_px=94.0f-weighted/available;
    result->valid=1u; result->track_lost=0u;
    return 1u;
}

// 函数简介     从底部可见道路选连续白段，再由地面边线法向恢复单边中线。
// 参数说明     binary只读；original用于帧元数据，不能凭原高可信绕过几何检查。
// 返回参数     通过地面和连续性约束的当前结果；无证据保持失效。
// 备注信息     42行以上不作地面证据；这是可配置软件ROI，仍需真实相机标定。
static void guard_road(const uint8_t *binary, const ControlParams *params,
                       const VisionResult *original, VisionResult *result)
{
    int row, previous_left=0, previous_right=187, active=0, gaps=0, has_single=0;
    float seed=94.0f;
    if(cached_result.valid && (uint32_t)(original->capture_time_ms-cached_result.capture_time_ms)<=150u)
        seed=94.0f-cached_result.error_px;
    *result=*original;
    memset(guarded_binary,0,sizeof(guarded_binary));
    memset(result->row_quality,0,sizeof(result->row_quality));
    memset(guard_kind,0,sizeof(guard_kind));
    result->valid=0u; result->track_lost=1u; result->confidence=0.0f;
    result->valid_rows=result->reconstructed_rows=0u; guard_single=0u;
    for(row=119;row>=(int)VISION_GROUND_TOP_ROW;--row) {
        int col=0, best_left=-1, best_right=-1;
        float best=1.0e9f;
        uint16_t reference=vision_get_reference_width((uint16_t)row);
        uint16_t expected=reference ? reference : learned_width[row];
        if(!reference && (uint32_t)(original->capture_time_ms-width_time[row])>1500u) expected=0u;
        while(col<188) {
            int left,right,width,overlap;
            float middle,distance;
            while(col<188 && binary[row*188+col]<=127u) ++col;
            left=col;
            while(col<188 && binary[row*188+col]>127u) ++col;
            right=col-1; width=right-left;
            if(width<3 || (left==0 && right==187)) continue;
            // 过窄白斑、突然展宽的地面亮块不能作为赛道双边；估计表只用作宽松筛选。
            if(left>0 && right<187) {
                unsigned model=expected ? expected : vision_map_widths[row];
                if(model && (width*100<(int)model*45 || width*100>(int)model*180)) continue;
            }
            overlap=(right<previous_right ? right : previous_right)-(left>previous_left ? left : previous_left);
            if(active && (overlap<2 || (left>0 && previous_left>0 && abs(left-previous_left)>12) ||
               (right<187 && previous_right<187 && abs(right-previous_right)>12))) continue;
            middle=0.5f*(float)(left+right);
            distance=seed<(float)left ? (float)left-seed : (seed>(float)right ? seed-(float)right : 0.0f);
            distance+=0.12f*fabsf(middle-seed);
            if(distance<best) { best=distance; best_left=left; best_right=right; }
        }
        if(best_left<0) {
            if(active && ++gaps>2) break;
            if(!active && row<=90) break; // 近端完全无路时不把远处墙面当成当前道路。
            continue;
        }
        {
            int left=best_left,right=best_right,width=right-left,middle=(left+right)/2;
            uint8_t quality=2u;
            active=1; gaps=0; previous_left=left; previous_right=right;
            memset(&guarded_binary[row*188+left],255,(size_t)(width+1));
            result->left[row]=(uint8_t)left; result->right[row]=(uint8_t)right;
            guard_kind[row]=3u;
            if(left==0 || right==187) {
                quality=0u;
                guard_kind[row]=(uint8_t)(left==0 ? 1u : 2u); has_single=1;
            }
            seed=(float)(quality ? middle : (left+right)/2);
            if(quality) {
                result->left[row]=(uint8_t)left; result->right[row]=(uint8_t)right;
                result->middle[row]=(uint8_t)middle; result->row_quality[row]=quality;
                ++result->valid_rows;
                // 只有居中且空间连续的双边观测更新短时宽度，单边和骨架不能自举标定。
                if(quality==2u && fabsf((float)middle-94.0f)<35.0f) {
                    learned_width[row]=(uint16_t)width; width_time[row]=original->capture_time_ms;
                }
            }
        }
    }
    if(has_single) {
        uint32_t stage=now_us();
        recover_normal(result);
        vision_timing_set(VISION_TIME_PROJECT,now_us()-stage); // 单边双向变换和法向恢复计入投影步骤，不重复计入原寻线。
    }
    (void)summarize_guard(params,result);
}

void vision_track_histogram(const uint8_t *gray, uint32_t *histogram)
{
    unsigned i;
    if(!histogram) return;
    memset(histogram,0,256u*sizeof(*histogram));
    if(!gray) return;
    for(i=VISION_GROUND_TOP_ROW*CONTROL_IMAGE_WIDTH;i<CONTROL_IMAGE_WIDTH*CONTROL_IMAGE_HEIGHT;++i)
        ++histogram[gray[i]];
}

static uint32_t now_us(void)
{
    return read_us != NULL ? read_us() : 0u;
}

static uint8_t over_budget(uint32_t started)
{
    return (uint8_t)(read_us != NULL && (uint32_t)(now_us() - started) > VISION_TRACK_BUDGET_US);
}

static float grid_x(uint16_t node)
{
    return 0.06f + 0.02f * (float)(node % VISION_TRACK_COLS);
}

static float grid_y(uint16_t node)
{
    return -1.15f + 0.02f * (float)(node / VISION_TRACK_COLS);
}

void vision_track_reset(void)
{
    memset(learned_width,0,sizeof(learned_width));
    memset(width_time,0,sizeof(width_time));
    frame_seen = history_valid = 0u; // 清历史不清地图，不修改任何控制或执行器状态。
    memset(&cached_result, 0, sizeof(cached_result));
}

void vision_track_init(uint32_t (*clock_us)(void))
{
    unsigned pattern, phase, i;
    read_us = clock_us;
    memset((void *)&vision_timing_log, 0, sizeof(vision_timing_log));
    memset(&timing_record, 0, sizeof(timing_record));
    vision_track_reset();
    memset(learned_width,0,sizeof(learned_width));
    memset(width_time,0,sizeof(width_time));
    // 顺序为上、右上、右、右下、下、左下、左、左上；与Zhang-Suen编号一致。
    for (pattern = 0u; pattern < 256u; ++pattern)
    {
        unsigned n[8], count = 0u, transitions = 0u;
        for (i = 0u; i < 8u; ++i) { n[i] = (pattern >> i) & 1u; count += n[i]; }
        for (i = 0u; i < 8u; ++i) transitions += (n[i] == 0u && n[(i + 1u) & 7u] != 0u);
        for (phase = 0u; phase < 2u; ++phase)
        {
            unsigned direction_ok = phase == 0u ?
                (n[0]*n[2]*n[4] == 0u && n[2]*n[4]*n[6] == 0u) :
                (n[0]*n[2]*n[6] == 0u && n[0]*n[4]*n[6] == 0u);
            delete_table[phase][pattern] = (uint8_t)(count >= 2u && count <= 6u && transitions == 1u && direction_ok);
        }
    }
}

void vision_timing_begin(uint32_t frame_id, uint32_t capture_ms, uint32_t started_us)
{
    memset(&timing_record, 0, sizeof(timing_record));
    timing_record.frame_id = frame_id;
    timing_record.capture_time_ms = capture_ms;
    timing_record.mapping_estimated = VISION_MAP_ESTIMATED;
    timing_record.clock_valid = (uint8_t)(read_us != NULL);
    timing_started_us = started_us;
}

void vision_timing_set(VisionTimingStage stage, uint32_t elapsed_us)
{
    if ((unsigned)stage < VISION_TIME_COUNT)
        timing_record.elapsed_us[stage] = read_us != NULL ? elapsed_us : 0u;
}

void vision_timing_finish(void)
{
    unsigned i;
    uint32_t slot = vision_timing_log.next;
    timing_record.finish_us = now_us();
    vision_timing_set(VISION_TIME_TOTAL, timing_record.finish_us - timing_started_us);
    ++vision_timing_log.sequence; // 奇数：外部不可读取半写快照；主循环仍保持中断开放。
    vision_timing_log.records[slot] = timing_record;
    for (i = 0u; i < VISION_TIME_COUNT; ++i)
    {
        uint32_t elapsed = timing_record.elapsed_us[i];
        vision_timing_log.sum_us[i] += elapsed;
        if (elapsed > vision_timing_log.max_us[i]) vision_timing_log.max_us[i] = elapsed;
    }
    vision_timing_log.next = (slot + 1u) % VISION_TIMING_CAPACITY;
    ++vision_timing_log.frames;
    ++vision_timing_log.sequence; // 偶数：整帧完成；外部还需核对前后sequence相同。
}

// 函数简介     固定网格的两阶段同步细化，边界之外统一视作背景。
// 参数说明     started为恢复开始微秒；road_mask已含当前帧白路。
// 返回参数     收敛为BASELINE，超时或未收敛分别返回BUDGET或LIMIT。
// 备注信息     收敛这里只表示可搜索，不代表已经获得有效视觉结果。
static uint8_t thin_road(uint32_t started)
{
    unsigned iteration, phase, row, col;
    static const int dr[8] = {-1,-1,0,1,1,1,0,-1}; // 八邻居的横向格偏移。
    static const int dc[8] = {0,1,1,1,0,-1,-1,-1}; // 八邻居的前向格偏移。
    memcpy(skeleton, road_mask, sizeof(skeleton));
    for (iteration = 0u; iteration < VISION_TRACK_ITERATIONS; ++iteration)
    {
        unsigned changed = 0u;
        for (phase = 0u; phase < 2u; ++phase)
        {
            memset(delete_mask, 0, sizeof(delete_mask));
            for (row = 0u; row < VISION_TRACK_ROWS; ++row)
            {
                for (col = 0u; col < VISION_TRACK_COLS; ++col)
                {
                    unsigned node = row*VISION_TRACK_COLS+col, pattern = 0u, k;
                    if (skeleton[node] == 0u) continue; // 背景点不用检查八个邻居。
                    for (k = 0u; k < 8u; ++k)
                    {
                        int y = (int)row+dr[k], x = (int)col+dc[k];
                        if (y >= 0 && y < (int)VISION_TRACK_ROWS && x >= 0 && x < (int)VISION_TRACK_COLS &&
                            skeleton[(unsigned)y*VISION_TRACK_COLS+(unsigned)x] != 0u) pattern |= 1u << k;
                    }
                    delete_mask[node] = delete_table[phase][pattern];
                    changed += delete_mask[node];
                }
                if (over_budget(started)) return VISION_TRACK_BUDGET; // 逐行有界检查，不等待整45轮才退出。
            }
            for (row = 0u; row < VISION_TRACK_CELLS; ++row)
                if (delete_mask[row] != 0u) skeleton[row] = 0u;
        }
        timing_record.thin_iterations = (uint16_t)(iteration+1u);
        if (changed == 0u) return VISION_TRACK_BASELINE;
    }
    return VISION_TRACK_LIMIT;
}

static uint8_t contains_node(const TrackPath *path, uint16_t node)
{
    unsigned i;
    for (i = 0u; i < path->count; ++i) if (path->nodes[i] == node) return 1u;
    return 0u;
}

static float rank_path(const TrackPath *path)
{
    uint16_t end = path->nodes[path->count-1u];
    float x = grid_x(end), y = grid_y(end);
    float continuity = history_valid ? 0.8f*hypotf(x-history_x, y-history_y) : 0.0f;
    // 前向延伸、横向不过度偏离和短时连续性共同评分；不把S弯强行当成直道。
    return path->score-x+0.8f*fabsf(y)-0.15f*path->length_m+continuity;
}

static void consider_path(const TrackPath *path, float *best)
{
    float score;
    if (path->length_m < 0.18f) return; // 太短的残留毛刺不能当作前进路径。
    score = rank_path(path);
    if (score < *best) { *best = score; chosen_path = *path; }
}

static void insert_beam(TrackPath *target, unsigned *count, const TrackPath *path)
{
    unsigned at, size = *count;
    // 只保留固定数量的低惩罚候选；每次插入最多移动8个固定结构体。
    for (at = 0u; at < size && target[at].score <= path->score; ++at) { }
    if (at >= VISION_TRACK_BEAMS) return;
    if (size < VISION_TRACK_BEAMS) ++size;
    if (size > at+1u) memmove(&target[at+1u], &target[at], (size-at-1u)*sizeof(*target));
    target[at] = *path;
    *count = size;
}

// 函数简介     从近处骨架根部进行有界八邻域候选追踪。
// 参数说明     started为恢复开始微秒。
// 返回参数     找到长度合格路径为RECOVERED，否则NO_PATH或BUDGET。
// 备注信息     最多65层、8候选、每层8邻居；重复节点及明显回头边被拒绝。
static uint8_t find_path(uint32_t started)
{
    unsigned node, count = 1u, current = 0u, step;
    float nearest = 1.0e9f, best = 1.0e9f;
    uint16_t root = VISION_MAP_INVALID;
    memset(&chosen_path, 0, sizeof(chosen_path));
    for (node = 0u; node < VISION_TRACK_CELLS; ++node)
    {
        if (skeleton[node] != 0u)
        {
            float x = grid_x((uint16_t)node)-0.42f, y = grid_y((uint16_t)node);
            float cost = x*x+2.0f*y*y;
            if (cost < nearest) { nearest = cost; root = (uint16_t)node; }
        }
    }
    if (root == VISION_MAP_INVALID || nearest > 0.25f) return VISION_TRACK_NO_PATH;
    memset(beams, 0, sizeof(beams));
    beams[0][0].nodes[0] = root;
    beams[0][0].count = 1u;
    beams[0][0].dir_col = 1.0f;
    for (step = 0u; step < 65u && count != 0u; ++step)
    {
        unsigned i, next_count = 0u, next = 1u-current;
        for (i = 0u; i < count; ++i)
        {
            TrackPath *path = &beams[current][i];
            uint16_t last = path->nodes[path->count-1u];
            int row = (int)(last/VISION_TRACK_COLS), col = (int)(last%VISION_TRACK_COLS), dr, dc;
            unsigned choices = 0u;
            if (path->length_m >= 1.05f || path->count >= VISION_TRACK_POINTS)
            {
                consider_path(path, &best);
                continue;
            }
            for (dr = -1; dr <= 1; ++dr)
            {
                for (dc = -1; dc <= 1; ++dc)
                {
                    int y = row+dr, x = col+dc;
                    uint16_t neighbor;
                    float norm, dot;
                    TrackPath child;
                    if ((dr == 0 && dc == 0) || (step < 3u && dc < 0) ||
                        y < 0 || y >= (int)VISION_TRACK_ROWS || x < 0 || x >= (int)VISION_TRACK_COLS) continue;
                    neighbor = (uint16_t)((unsigned)y*VISION_TRACK_COLS+(unsigned)x);
                    if (skeleton[neighbor] == 0u || contains_node(path, neighbor)) continue;
                    norm = dr != 0 && dc != 0 ? 1.41421356f : 1.0f;
                    dot = ((float)dr*path->dir_row+(float)dc*path->dir_col)/norm;
                    if (dot < -0.35f) continue;
                    child = *path;
                    child.nodes[child.count++] = neighbor;
                    child.score += 0.12f*(1.0f-dot);
                    child.length_m += norm*0.02f;
                    child.dir_row = (float)dr/norm;
                    child.dir_col = (float)dc/norm;
                    insert_beam(beams[next], &next_count, &child);
                    ++choices;
                }
            }
            if (choices == 0u) consider_path(path, &best);
        }
        if (over_budget(started)) return VISION_TRACK_BUDGET;
        current = next;
        count = next_count;
    }
    for (node = 0u; node < count; ++node) consider_path(&beams[current][node], &best);
    timing_record.path_points = chosen_path.count;
    return chosen_path.count >= 2u ? VISION_TRACK_RECOVERED : VISION_TRACK_NO_PATH;
}

static unsigned append_projection(unsigned count, uint16_t node)
{
    uint16_t pixel = vision_map_indices[node];
    if (pixel == VISION_MAP_INVALID || count >= VISION_TRACK_POINTS+25u) return count;
    projected_u[count] = (float)(pixel%CONTROL_IMAGE_WIDTH);
    projected_v[count] = (float)(pixel/CONTROL_IMAGE_WIDTH);
    return count+1u;
}

// 函数简介     将观测骨架路径投影回地面ROI，再选择有效控制带，仍输出像素误差。
// 参数说明     params提供控制带；result包含原帧元数据及原边线。
// 返回参数     至少三行且质量加权达到0.35时为1，否则0。
// 备注信息     近端切线最多延伸0.48米，逐格必须有当前白路支持，空图不能靠历史伪造中线。
static uint8_t project_path(const ControlParams *params, VisionResult *result)
{
    unsigned count = 0u, i, rows = 0u;
    VisionResult candidate = *result;
    float weight_sum = 0.0f, available = 0.0f, weighted = 0.0f;
    uint16_t root = chosen_path.nodes[0], tangent_node = chosen_path.nodes[chosen_path.count > 5u ? 5u : chosen_path.count-1u];
    float dx = grid_x(tangent_node)-grid_x(root), dy = grid_y(tangent_node)-grid_y(root);
    float norm = hypotf(dx, dy);
    uint16_t near_nodes[24]; // 从根部向后最多24格，延伸只补近端，不跨越未观察白路。
    unsigned near_count = 0u;
    if (norm > 0.001f)
    {
        for (i = 1u; i <= 24u; ++i)
        {
            float x = grid_x(root)-0.02f*(float)i*dx/norm, y = grid_y(root)-0.02f*(float)i*dy/norm;
            int col = (int)floorf((x-0.06f)/0.02f+0.5f), row = (int)floorf((y+1.15f)/0.02f+0.5f);
            unsigned node;
            if (row < 0 || row >= (int)VISION_TRACK_ROWS || col < 0 || col >= (int)VISION_TRACK_COLS) break;
            node = (unsigned)row*VISION_TRACK_COLS+(unsigned)col;
            if (road_mask[node] == 0u) break;
            near_nodes[near_count++] = (uint16_t)node;
        }
    }
    while (near_count != 0u) count = append_projection(count, near_nodes[--near_count]);
    for (i = 0u; i < chosen_path.count; ++i) count = append_projection(count, chosen_path.nodes[i]);
    memset(candidate.row_quality, 0, sizeof(candidate.row_quality));
    candidate.valid_rows = candidate.reconstructed_rows = 0u;
    // 逐行求交后统一选带；单边必须由法向恢复，不能用被画面裁剪的白路骨架替代。
    for (i = VISION_GROUND_TOP_ROW; i < CONTROL_IMAGE_HEIGHT; ++i)
    {
        unsigned k, row = i;
        float center = 0.0f, closest = 1.0e9f, expected = result->row_quality[row] ? result->middle[row] : 94.0f;
        int found = 0;
        float weight = 1.0f; // 完整路径逐行生成，最后再按实际控制带加权。
        weight_sum += weight;
        for (k = 1u; k < count; ++k)
        {
            float a = projected_v[k-1u], b = projected_v[k];
            if ((a-(float)row)*(b-(float)row) <= 0.0f && fabsf(b-a) > 0.001f)
            {
                float u = projected_u[k-1u]+(projected_u[k]-projected_u[k-1u])*((float)row-a)/(b-a);
                float distance = fabsf(u-expected);
                if (u >= 0.0f && u <= 187.0f && distance < closest) { closest = distance; center = u; found = 1; }
            }
        }
        if (found && guard_kind[row]==3u && guarded_binary[row*188u+(unsigned)floorf(center+0.5f)])
        {
            int middle = (int)floorf(center+0.5f);
            float quality = 0.5f;
            candidate.middle[row] = (uint8_t)middle;
            candidate.left[row] = result->left[row]; candidate.right[row] = result->right[row]; // 保留当前真实白段，不生成假双边。
            candidate.row_quality[row] = 1u; // 默认全部是路径重建，可信度上限0.5。
            if (result->row_quality[row] == 2u && fabsf(center-(float)result->middle[row]) <= 4.0f)
            {
                candidate.left[row] = result->left[row]; candidate.right[row] = result->right[row];
                candidate.row_quality[row] = 2u; quality = 1.0f; // 与双实测中点一致才沿用原边界。
            }
            available += weight*quality;
            weighted += weight*quality*(float)middle;
            ++rows; ++candidate.valid_rows;
            if (candidate.row_quality[row] == 1u) ++candidate.reconstructed_rows;
        }
    }
    if (rows < 3u || available <= 0.0f || available/weight_sum < 0.35f) return 0u;
    candidate.valid=0u; candidate.track_lost=1u;
    if(!summarize_guard(params,&candidate)) return 0u;
    (void)weighted; // 误差和置信度由实际选择的连续控制带给出。
    candidate.valid = 1u; candidate.track_lost = 0u;
    *result = candidate; // 帧号、采集时间和阈值来自原结果，绝不刷新成现在。
    return 1u;
}

uint8_t vision_track_process(const uint8_t *binary, const ControlParams *params, VisionResult *result)
{
    unsigned i, white = 0u, visible = 0u;
    uint8_t status = VISION_TRACK_BASELINE, strong = 1u;
    uint32_t started = now_us(), stage;
    float total_weight = 0.0f;
    VisionResult original, guarded;
    timing_record.recovery_status = VISION_TRACK_INVALID;
    if (binary == NULL || params == NULL || result == NULL) return VISION_TRACK_INVALID;
    original = *result;
    for (i = 0u; i < CONTROL_BAND_COUNT; ++i)
    {
        unsigned row = params->look_rows[i];
        uint16_t reference;
        if (row >= CONTROL_IMAGE_HEIGHT || !isfinite(params->look_weights[i]) || params->look_weights[i] <= 0.0f ||
            (i != 0u && row <= params->look_rows[i-1u])) return VISION_TRACK_INVALID;
        total_weight += params->look_weights[i];
        reference = vision_get_reference_width((uint16_t)row);
        if (result->row_quality[row] != 2u || (reference != 0u &&
            (float)((int)result->right[row]-(int)result->left[row]) > 1.35f*(float)reference)) strong = 0u;
    }
    if (!isfinite(total_weight) || total_weight <= 0.0f) return VISION_TRACK_INVALID;
    if (frame_seen && (int32_t)(result->frame_id-last_frame_id) <= 0)
    {
        if (result->frame_id == last_frame_id) *result = cached_result;
        timing_record.recovery_status = VISION_TRACK_DUPLICATE;
        return VISION_TRACK_DUPLICATE;
    }
    memcpy(vision_track_control_rows,params->look_rows,CONTROL_BAND_COUNT);
    last_frame_id = result->frame_id; frame_seen = 1u;
    if (history_valid && (uint32_t)(result->capture_time_ms-history_ms) > 100u) history_valid = 0u;
    stage=now_us();
    guard_road(binary,params,result,&guarded);
    vision_timing_set(VISION_TIME_BASELINE,timing_record.elapsed_us[VISION_TIME_BASELINE]+now_us()-stage-timing_record.elapsed_us[VISION_TIME_PROJECT]);
    if(over_budget(started)) {
        *result=original; result->valid=0u; result->track_lost=1u; result->confidence=0.0f;
        status=VISION_TRACK_BUDGET; goto finished; // 处理超时不得发布未完成验证的有效结果。
    }
    // 原高可信还必须与近端连续道路一致；错误白段不能直接跳过恢复。
    if(guarded.valid && guarded.confidence>=0.99f && result->valid && result->confidence>=0.8f && strong &&
       fabsf(guarded.error_px-result->error_px)<=4.0f && !guard_adaptive && !guard_single) {
        *result=guarded; // 前瞻会读取近带以外的行；即使近带强可信，也必须返回整幅经过连续道路筛选的数据。
        goto finished;
    }
    *result=guarded;
    original=guarded; // 后续超时只回退到已经过几何检查的结果，不能恢复被拒绝的假中线。
    if(guarded.valid) {
        status=guard_adaptive ? VISION_TRACK_ADAPTIVE : (guard_single ? VISION_TRACK_SINGLE : VISION_TRACK_GUARDED);
        goto finished;
    }
    stage = now_us();
    for (i = 0u; i < VISION_TRACK_CELLS; ++i)
    {
        uint16_t pixel = vision_map_indices[i];
        road_mask[i] = (uint8_t)(pixel != VISION_MAP_INVALID && guarded_binary[pixel] > 127u);
        white += road_mask[i]; visible += pixel != VISION_MAP_INVALID;
    }
    vision_timing_set(VISION_TIME_BEV, now_us()-stage);
    if (white < 20u || white*10u > visible*8u) { status = VISION_TRACK_NO_PATH; goto finished; }
    if (over_budget(started)) { status = VISION_TRACK_BUDGET; goto finished; }
    stage = now_us(); status = thin_road(started);
    vision_timing_set(VISION_TIME_THIN, now_us()-stage);
    if (status != VISION_TRACK_BASELINE) goto finished;
    stage = now_us(); status = find_path(started);
    vision_timing_set(VISION_TIME_PATH, now_us()-stage);
    if (status != VISION_TRACK_RECOVERED) goto finished;
    stage = now_us();
    if (!project_path(params, result)) status = VISION_TRACK_NO_PATH;
    vision_timing_set(VISION_TIME_PROJECT, timing_record.elapsed_us[VISION_TIME_PROJECT]+now_us()-stage);
    if (over_budget(started)) { *result = original; status = VISION_TRACK_BUDGET; }
    if (status == VISION_TRACK_RECOVERED)
    {
        uint16_t end = chosen_path.nodes[chosen_path.count-1u];
        history_x = grid_x(end); history_y = grid_y(end);
        history_ms = result->capture_time_ms; history_valid = 1u;
    }
finished:
    if (status != VISION_TRACK_RECOVERED && status != VISION_TRACK_BASELINE) history_valid = 0u;
    cached_result = *result;
    timing_record.recovery_status = status;
    return status;
}
