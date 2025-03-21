#include <imgui/imgui.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui/imgui_internal.h>

namespace ImGui {

static ImU32 InvertColorU32(ImU32 in)
{
    ImVec4 in4 = ColorConvertU32ToFloat4(in);
    in4.x = 1.f - in4.x;
    in4.y = 1.f - in4.y;
    in4.z = 1.f - in4.z;
    return GetColorU32(in4);
}

static void PlotMultiEx(
    ImGuiPlotType plot_type,
    const char* label,
    int num_datas,
    const char** names,
    const ImColor* colors,
    float(*getter)(const void* data, int idx),
	const void * (*data_getter)(const void * const * datas, int data_idx),
    const void * const * datas,
    int values_count,
	int values_display,
	int values_offset,
    float scale_min,
    float scale_max,
	ImVec2 graph_size,
	bool logScale = true)
{
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
        return;

    ImGuiContext& g = *GImGui;
    const ImGuiStyle& style = g.Style;

    const ImVec2 label_size = ImGui::CalcTextSize(label, NULL, true);
    if (graph_size.x == 0.0f)
        graph_size.x = CalcItemWidth();
    if (graph_size.y == 0.0f)
        graph_size.y = label_size.y + (style.FramePadding.y * 2);

    const ImRect frame_bb(window->DC.CursorPos, window->DC.CursorPos + ImVec2(graph_size.x, graph_size.y));
    const ImRect inner_bb(frame_bb.Min + style.FramePadding, frame_bb.Max - style.FramePadding);
	const ImRect total_bb(frame_bb.Min, frame_bb.Max + ImVec2(label_size.x > 0.0f ? style.ItemInnerSpacing.x + label_size.x : 0.0f, 0));
    ItemSize(total_bb, style.FramePadding.y);
	if (!ItemAdd(total_bb, 0))
        return;

    // Determine scale from values if not specified
    if (scale_min == FLT_MAX || scale_max == FLT_MAX)
    {
        float v_min = FLT_MAX;
        float v_max = -FLT_MAX;
        for (int data_idx = 0; data_idx < num_datas; ++data_idx)
        {
            for (int i = 0; i < values_count; i++)
            {
				const float v = getter(data_getter(datas, data_idx), i);
                v_min = ImMin(v_min, v);
                v_max = ImMax(v_max, v);
            }
        }
        if (scale_min == FLT_MAX)
            scale_min = v_min;
        if (scale_max == FLT_MAX)
            scale_max = v_max;
    }

    RenderFrame(frame_bb.Min, frame_bb.Max, GetColorU32(ImGuiCol_FrameBg), true, style.FrameRounding);

	int res_w = ImMin((int) graph_size.x, values_display) + ((plot_type == ImGuiPlotType_Lines) ? -1 : 0);
	int item_count = values_display + ((plot_type == ImGuiPlotType_Lines) ? -1 : 0);

	// Tooltip on hover
	int v_hovered = -1;
	if (ItemHoverable(inner_bb, 0))
	{
		const float t = ImClamp((g.IO.MousePos.x - inner_bb.Min.x) / (inner_bb.Max.x - inner_bb.Min.x), 0.0f, 0.9999f);
		const int v_idx = (int) (t * item_count);
		IM_ASSERT(v_idx >= 0 && v_idx < values_count);

		// std::string toolTip;
		ImGui::BeginTooltip();
		const int idx0 = (v_idx + values_offset) % values_count;
		if (plot_type == ImGuiPlotType_Lines)
		{
			const int idx1 = (v_idx + 1 + values_offset) % values_count;
			Text("%8d %8d | Name", v_idx, v_idx+1);
			for (int dataIdx = 0; dataIdx < num_datas; ++dataIdx)
			{
				const float v0 = getter(datas[dataIdx], idx0);
				const float v1 = getter(datas[dataIdx], idx1);
				TextColored(colors[dataIdx], "%8.4g %8.4g | %s", v0, v1, names[dataIdx]);
			}
		}
		else if (plot_type == ImGuiPlotType_Histogram)
		{
			for (int dataIdx = 0; dataIdx < num_datas; ++dataIdx)
			{
				const float v0 = getter(datas[dataIdx], idx0);
				TextColored(colors[dataIdx], "%d: %8.4g | %s", v_idx, v0, names[dataIdx]);
			}
		}
		ImGui::EndTooltip();
		v_hovered = v_idx;
	}

	auto scale = [scale_min, scale_max, logScale](float v){
		return (logScale) ? log2f(v-scale_min+1)/log2f(scale_max-scale_min+1) : (v - scale_min) / (scale_max - scale_min);
	};

    for (int data_idx = 0; data_idx < num_datas; ++data_idx)
    {
        const float t_step = 1.0f / (float) res_w;

        float v0 = getter(datas[data_idx], (0 + values_offset) % values_count);
        float t0 = 0.0f;
		const ImVec2 tp1 = ImVec2(t0, 1.0f - ImSaturate(scale(v0)));
		ImVec2 tp0 = ImVec2(t0, 1.0f - ImSaturate(scale(v0)));    // Point in the normalized space of our target rectangle

        const ImU32 col_base = colors[data_idx];
        const ImU32 col_hovered = InvertColorU32(colors[data_idx]);

        //const ImU32 col_base = GetColorU32((plot_type == ImGuiPlotType_Lines) ? ImGuiCol_PlotLines : ImGuiCol_PlotHistogram);
        //const ImU32 col_hovered = GetColorU32((plot_type == ImGuiPlotType_Lines) ? ImGuiCol_PlotLinesHovered : ImGuiCol_PlotHistogramHovered);

        for (int n = 0; n < res_w; n++)
        {
            const float t1 = t0 + t_step;
			const int v1_idx = (int) (t0 * item_count + 0.5f);
            IM_ASSERT(v1_idx >= 0 && v1_idx < values_count);
			const float v1 = getter(datas[data_idx], (v1_idx + values_offset + 1) % values_count);
			const ImVec2 tp1 = ImVec2(t1, 1.0f - ImSaturate(scale(v1)));

            // NB: Draw calls are merged together by the DrawList system. Still, we should render our batch are lower level to save a bit of CPU.
            ImVec2 pos0 = ImLerp(inner_bb.Min, inner_bb.Max, tp0);
            ImVec2 pos1 = ImLerp(inner_bb.Min, inner_bb.Max, (plot_type == ImGuiPlotType_Lines) ? tp1 : ImVec2(tp1.x, 1.0f));
            if (plot_type == ImGuiPlotType_Lines)
            {
                window->DrawList->AddLine(pos0, pos1, v_hovered == v1_idx ? col_hovered : col_base);
            }
            else if (plot_type == ImGuiPlotType_Histogram)
            {
                if (pos1.x >= pos0.x + 2.0f)
                    pos1.x -= 1.0f;
                window->DrawList->AddRectFilled(pos0, pos1, v_hovered == v1_idx ? col_hovered : col_base);
            }

            t0 = t1;
            tp0 = tp1;
        }
    }

    RenderText(ImVec2(frame_bb.Max.x + style.ItemInnerSpacing.x, inner_bb.Min.y), label);
}

struct ImGuiPlotArrayGetterData
{
	const void * const * Values;
	int Stride;

	ImGuiPlotArrayGetterData(const void* const * values, int stride) { Values = values; Stride = stride; }
};

static const void * Plot_ArrayGetter(const void * const * data, int idx)
{
	ImGuiPlotArrayGetterData* plot_data = (ImGuiPlotArrayGetterData*)data;
	const void * v = (const void*)(void*)((unsigned char*)plot_data->Values + (size_t)idx * plot_data->Stride);
	return v;
}

void PlotMultiLines(
	const char* label,
	int num_datas,
	const char** names,
	const ImColor* colors,
	float(*getter)(const void* data, int idx),
	const void * (*data_getter)(const void* const * datas, int idx),
	const void * const * datas,
	int values_count,
	int values_display,
	int values_offset,
	float scale_min,
	float scale_max,
	ImVec2 graph_size)
{
	PlotMultiEx(ImGuiPlotType_Lines, label, num_datas, names, colors, getter, data_getter, datas, values_count, values_display, values_offset, scale_min, scale_max, graph_size);
}

void PlotMultiLines(
	const char* label,
	int num_datas,
	const char** names,
	const ImColor* colors,
	float(*getter)(const void* data, int idx),
	int stride,
	const void * const * datas,
	int values_count,
	int values_display,
	int values_offset,
	float scale_min,
	float scale_max,
	ImVec2 graph_size)
{
	ImGuiPlotArrayGetterData data_getter(datas, stride);
	PlotMultiEx(ImGuiPlotType_Lines, label, num_datas, names, colors, getter, &Plot_ArrayGetter, (const void* const *)&data_getter, values_count, values_display, values_offset, scale_min, scale_max, graph_size);
}

void PlotMultiLines(
    const char* label,
    int num_datas,
    const char** names,
    const ImColor* colors,
    float(*getter)(const void* data, int idx),
    const void * const * datas,
    int values_count,
	int values_display,
	int values_offset,
    float scale_min,
    float scale_max,
    ImVec2 graph_size)
{
	PlotMultiEx(ImGuiPlotType_Lines, label, num_datas, names, colors, getter, &Plot_ArrayGetter, datas, values_count, values_display, values_offset, scale_min, scale_max, graph_size);
}

void PlotMultiHistograms(
    const char* label,
    int num_hists,
    const char** names,
    const ImColor* colors,
    float(*getter)(const void* data, int idx),
    const void * const * datas,
    int values_count,
	int values_display,
	int values_offset,
    float scale_min,
    float scale_max,
    ImVec2 graph_size)
{
	PlotMultiEx(ImGuiPlotType_Histogram, label, num_hists, names, colors, getter, &Plot_ArrayGetter, datas, values_count, values_display, values_offset, scale_min, scale_max, graph_size);
}

} // namespace ImGui
