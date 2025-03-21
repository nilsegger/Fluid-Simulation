#include "Gui.h"
#include <igl/Hit.h>
#include <igl/project.h>
#include <igl/writeOFF.h>
#include <iomanip>
#include <sstream>

void Gui::setSimulation(Simulation *sim) {
	p_simulator = new Simulator(sim);
	p_simulator->reset();
	p_simulator->setSimulationSpeed(m_simSpeed);
}


void Gui::start() {
	// message: http://patorjk.com/software/taag/#p=display&v=0&f=Roman&t=PBS%2019
	std::string usage(
		R"(
          ooooooooo.   oooooooooo.   .oooooo..o
          `888   `Y88. `888'   `Y8b d8P'    `Y8
           888   .d88'  888     888 Y88bo.     
           888ooo88P'   888oooo888'  `"Y8888o. 
           888          888    `88b      `"Y88b
           888          888    .88P oo     .d8P
          o888o        o888bood8P'  8""88888P'

  252-0546-00L Physically-Based Simulation in Computer Graphics @ ETH Zurich
  Course Exercise Framework
  
  Shortcuts:
  [drag] Rotate scene                 |  [space] Start/pause simulation
  I,i    Toggle invert normals        |  A,a     Single step
  L,l    Toggle wireframe             |  R,r     Reset simulation
  T,t    Toggle filled faces          |  C,c     Clear screen
  ;      Toggle vertex labels         |  :       Toggle face labels
  -      Toggle fast forward          |  Z       Snap to canonical view
  .      Turn up lighting             |  ,       Turn down lighting
  O,o    Toggle orthographic/perspective projection)");
	std::cout << usage << std::endl;

	// setting up viewer
	m_viewer.data().show_lines = false;
	m_viewer.data().point_size = 5.0f;
	m_viewer.core.background_color.setOnes();
    m_viewer.data().set_face_based(true);
    m_viewer.data().shininess = 1.0;
	m_viewer.core.is_animating = true;
	m_viewer.core.camera_zoom = 0.1;
	m_viewer.core.object_scale = 1.0;

	// setting up menu
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	m_viewer.plugins.push_back(&menu);
	menu.callback_draw_viewer_window = [&]() { drawMenuWindow(menu); };
	showAxes(m_showAxes);
	p_simulator->setNumRecords(m_numRecords);
	p_simulator->setMaxSteps(m_maxSteps);

	// callbacks
	m_viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &viewer,
		unsigned int key, int modifiers) {
		return keyCallback(viewer, key, modifiers);
	};

	m_viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &viewer) {
		return drawCallback(viewer);
	};

	m_viewer.callback_mouse_scroll = [&](igl::opengl::glfw::Viewer &viewer,
		float delta_y) {
		return scrollCallback(viewer, delta_y);
	};

	m_viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer &viewer,
		int button, int modifier) {
		return mouseCallback(viewer, menu, button, modifier);
	};

	// start viewer
	m_viewer.launch();
}

void Gui::resetSimulation() {
	p_simulator->reset();
	m_timerAverage = 0.0;
}

#pragma region ArrowInterface
int Gui::addArrow(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
	const Eigen::Vector3d &color) {
	m_arrows.push_back(Arrow(start, end, color));
	m_arrows.back().id = m_numArrows++;
	return m_arrows.back().id;
}

void Gui::removeArrow(size_t index) {
	bool found = false;
	for (size_t i = 0; i < m_arrows.size(); i++) {
		if (m_arrows[i].id == index) {
			found = true;
			m_arrows.erase(m_arrows.begin() + i);
		}
	}
	assert(found && "unable to find index");
}

void Gui::drawArrow(const Arrow &arrow) {
	m_viewer.data_list[0].add_edges(arrow.start, arrow.end, arrow.color);
	m_viewer.data_list[0].add_edges(arrow.end, arrow.head[0], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.end, arrow.head[1], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.end, arrow.head[2], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.end, arrow.head[3], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.head[0], arrow.head[2], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.head[1], arrow.head[2], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.head[1], arrow.head[3], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.head[3], arrow.head[0], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.head[3], arrow.head[2], arrow.color);
	m_viewer.data_list[0].add_edges(arrow.head[1], arrow.head[0], arrow.color);
}
#pragma endregion ArrowInterface

void Gui::drawReferencePlane() {
	m_viewer.data_list[0].add_edges(m_referencePlane.start, m_referencePlane.end, m_referencePlane.color);
}

void Gui::showVertexArrow() {
	if (m_clickedArrow >= 0) {
		removeArrow(m_clickedArrow);
		m_clickedArrow = -1;
	}
	if (m_clickedVertex >= 0) {
		Eigen::Vector3d pos;
		Eigen::Vector3d norm;
		if (callback_clicked_vertex) {
			callback_clicked_vertex(m_clickedVertex, m_clickedObject, pos,
				norm);
		}
		else {
			pos = m_viewer.data_list[m_clickedObject].V.row(m_clickedVertex);
			norm = m_viewer.data_list[m_clickedObject].V_normals.row(
				m_clickedVertex);
		}
		m_clickedArrow =
			addArrow(pos, pos + norm, Eigen::RowVector3d(1.0, 0, 0));
	}
}

bool Gui::drawCallback(igl::opengl::glfw::Viewer &viewer) {
	if (m_request_clear) {
		for (auto &d : viewer.data_list) {
			d.clear();
		}
		m_request_clear = false;
		m_clickedVertex = -1;
		if (m_clickedArrow >= 0) {
			removeArrow(m_clickedArrow);
			m_clickedArrow = -1;
		}
	}

	viewer.data_list[0].clear();
	if (m_arrows.size() > 0 || m_showReferencePlane) {
		for (size_t i = 0; i < m_arrows.size(); i++) {
			drawArrow(m_arrows[i]);
		}

		if (m_showReferencePlane) drawReferencePlane();
	}

	p_simulator->render(viewer);

	
	return false;
}

bool Gui::scrollCallback(igl::opengl::glfw::Viewer &viewer, float delta_y) {
	double factor = 1.5;
	if (delta_y > 0)
		viewer.core.camera_zoom *= factor;
	else
		viewer.core.camera_zoom /= factor;
	return true;
}

void Gui::toggleSimulation() {
	if (p_simulator->isPaused()) {
		if (!p_simulator->hasStarted()) {
			updateSimulationParameters();
		}
		p_simulator->run();
	}
	else {
		p_simulator->pause();
	}
}

void Gui::singleStep() {
	if (!p_simulator->hasStarted()) {
		updateSimulationParameters();
	}
	p_simulator->run(true);
}

void Gui::clearScreen() {
	m_request_clear = true;
	clearSimulation();
}

bool Gui::keyCallback(igl::opengl::glfw::Viewer &viewer, unsigned int key,
	int modifiers) {
	switch (key) {
	case 'I':
	case 'i':
		for (auto &d : viewer.data_list) {
			d.dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			d.invert_normals = !d.invert_normals;
		}
		return true;
	case 'L':
	case 'l':
		for (auto &d : viewer.data_list) {
			d.show_lines = !d.show_lines;
		}
		return true;
	case 'T':
	case 't':
		for (auto &d : viewer.data_list) {
			d.show_faces = !d.show_faces;
		}
		return true;
	case ';':
		for (auto &d : viewer.data_list) {
			d.show_vertid = !d.show_vertid;
		}
		return true;
	case ':':
		for (auto &d : viewer.data_list) {
			d.show_faceid = !d.show_faceid;
		}
		return true;
	case ' ':
		toggleSimulation();
		return true;
	case 'r':
	case 'R':
		resetSimulation();
		return true;
	case 'a':
	case 'A':
		singleStep();
	case 'c':
	case 'C':
		clearScreen();
		return true;
	case '-':
		setFastForward(!m_fastForward);
		return true;
	case '.':
		viewer.core.lighting_factor += 0.1;
		break;
	case ',':
		viewer.core.lighting_factor -= 0.1;
		break;
	default:
		return childKeyCallback(viewer, key, modifiers);
	}
	viewer.core.lighting_factor =
		std::min(std::max(viewer.core.lighting_factor, 0.f), 1.f);
	return false;
}

bool Gui::mouseCallback(igl::opengl::glfw::Viewer &viewer,
	igl::opengl::glfw::imgui::ImGuiMenu &menu, int button,
	int modifier) {
	// get vertices, project them onto screen and find closest vertex to mouse
	float minDist = std::numeric_limits<float>::infinity();
	int vertex = -1;
	int object = -1;
	for (size_t i = 0; i < viewer.data_list.size(); i++) {
		Eigen::MatrixXf Vf = viewer.data_list[i].V.cast<float>();
		Eigen::MatrixXf projections;
		igl::project(Vf, viewer.core.view, viewer.core.proj,
			viewer.core.viewport, projections);

		Eigen::VectorXf x = projections.col(0).array() - viewer.current_mouse_x;
		Eigen::VectorXf y = -projections.col(1).array() +
			viewer.core.viewport(3) - viewer.current_mouse_y;
		Eigen::VectorXf distances =
			(x.array().square() + y.array().square()).matrix().cwiseSqrt();

		int vi = -1;
		float dist = distances.minCoeff(&vi);
		if (dist < minDist) {
			minDist = dist;
			vertex = vi;
			object = i;
		}
	}
	if (minDist < 20) {
		// only select vertex if user clicked "close"
		m_clickedVertex = vertex;
		m_clickedObject = object;
		showVertexArrow();
	}
	return false;
}

void Gui::drawMenuWindow(igl::opengl::glfw::imgui::ImGuiMenu &menu) {
	glfwSetWindowTitle(m_viewer.window, "PBS Exercises");

	float menu_width = 220.f * menu.menu_scaling();

	// Controls
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
	ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
		ImVec2(menu_width, -1.0f));
	bool _viewer_menu_visible = true;
	ImGui::Begin(
		"Viewer", &_viewer_menu_visible,
		ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
	ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
	drawMenu(m_viewer, menu);
	ImGui::PopItemWidth();
	ImGui::End();

	// Clicking
	if (m_clickedVertex >= 0) {
		ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize,
			ImGuiSetCond_FirstUseEver);
		bool visible = true;
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
		ImGui::Begin(
			"ViewerLabels", &visible,
			ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
			ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoScrollWithMouse |
			ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
			ImGuiWindowFlags_NoInputs);

		Eigen::Vector3d pos =
			m_viewer.data_list[m_clickedObject].V.row(m_clickedVertex);
		Eigen::Vector3d norm =
			m_viewer.data_list[m_clickedObject].V_normals.row(m_clickedVertex);
		std::string text = "(" + std::to_string(pos(0)) + ", " +
			std::to_string(pos(1)) + ", " +
			std::to_string(pos(2)) + ")";

		ImDrawList *drawList = ImGui::GetWindowDrawList();
		Eigen::Vector3f c0 = igl::project(
			Eigen::Vector3f((pos + 0.1 * norm).cast<float>()),
			m_viewer.core.view, m_viewer.core.proj, m_viewer.core.viewport);
		drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.2,
			ImVec2(c0(0), m_viewer.core.viewport[3] - c0(1)),
			ImGui::GetColorU32(ImVec4(0, 0, 10, 255)), &text[0],
			&text[0] + text.size());

		ImGui::End();
		ImGui::PopStyleColor();
		ImGui::PopStyleVar();

		showVertexArrow();
	}

	// Stats
	if (m_showStats) {
		int width, height;
		glfwGetWindowSize(m_viewer.window, &width, &height);
		ImGui::SetNextWindowPos(ImVec2(width - menu_width, 0.0f),
			ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
			ImVec2(menu_width, -1.0f));
		ImGui::Begin("Stats", &_viewer_menu_visible,
			ImGuiWindowFlags_NoSavedSettings |
			ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
		if (!p_simulator->isPaused()) {
			if (m_timerAverage > 0.0) {
				double alpha = 0.95;
				m_timerAverage = m_timerAverage * alpha +
					(1.0 - alpha) * p_simulator->getDuration();
			}
			else {
				m_timerAverage = p_simulator->getDuration();
			}
		}
		ImGui::Text("Iteration: %ld", p_simulator->getSimulationStep());
		ImGui::Text("Average time per iteration: %.2fms", m_timerAverage);
		ImGui::Text("Current time: %.5f", p_simulator->getSimulationTime());
		drawSimulationStats();
		ImGui::PopItemWidth();
		ImGui::End();
	}
}

inline std::string getFilename(int total_numObj, int obj, int total_steps,
	int step) {
	std::stringstream ss;
	ss << "_object" << std::setw(std::log10(total_numObj)) << std::setfill('0')
		<< obj << "_" << std::setw(std::log10(total_steps)) << step << ".obj";
	return ss.str();
}

void Gui::exportRecording() {
	std::string path = igl::file_dialog_save();
	size_t finddot = path.find_last_of(".");
	path = path.substr(0, finddot);
	std::cout << "Exporting Recording to " << path << "_objectxxx_xxx.obj"
		<< std::endl;
	auto rec = p_simulator->getRecords();
	int steps = rec[0].size();
	for (size_t i = 0; i < rec.size(); i++) {
		int j = 0;
		while (rec[i].size() > 0) {
			std::string filename =
				path + getFilename(rec.size(), i, steps, j++);
			auto p = rec[i].front();
			rec[i].pop();
			bool succ = igl::writeOBJ(filename, p.first, p.second);
			if (!succ) {
				std::cerr << "Failed to write recording" << std::endl;
			}
		}
	}
}

bool Gui::drawMenu(igl::opengl::glfw::Viewer &viewer,
	igl::opengl::glfw::imgui::ImGuiMenu &menu) {
	if (ImGui::CollapsingHeader("Simulation Control",
		ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::Button(
			p_simulator->isPaused() ? "Run Simulation" : "Pause Simulation",
			ImVec2(-1, 0))) {
			toggleSimulation();
		}
		if (ImGui::Button("Single Step", ImVec2(-1, 0))) {
			singleStep();
		}
		if (ImGui::Button("Reset Simulation", ImVec2(-1, 0))) {
			resetSimulation();
		}
		if (ImGui::Button("Clear Screen", ImVec2(-1, 0))) {
			clearScreen();
		}
		if (ImGui::SliderInt("Steps/Second", &m_simSpeed, 1, m_maxSimSpeed)) {
			p_simulator->setSimulationSpeed(m_simSpeed);
		}
		if (ImGui::InputInt("Max Steps", &m_maxSteps, -1, -1)) {
			p_simulator->setMaxSteps(m_maxSteps);
		}
	}
	if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::Checkbox("Wireframe", &(viewer.data().show_lines))) {
			for (size_t i = 0; i < viewer.data_list.size(); i++) {
				viewer.data_list[i].show_lines = viewer.data().show_lines;
			}
		}
		if (ImGui::Checkbox("Fill", &(viewer.data().show_faces))) {
			for (size_t i = 0; i < viewer.data_list.size(); i++) {
				viewer.data_list[i].show_faces = viewer.data().show_faces;
			}
		}
		if (ImGui::Checkbox("Show vertex labels",
			&(viewer.data().show_vertid))) {
			for (size_t i = 0; i < viewer.data_list.size(); i++) {
				viewer.data_list[i].show_vertid = viewer.data().show_vertid;
			}
		}
		if (ImGui::Checkbox("Show faces labels",
			&(viewer.data().show_faceid))) {
			for (size_t i = 0; i < viewer.data_list.size(); i++) {
				viewer.data_list[i].show_faceid = viewer.data().show_faceid;
			}
		}
		ImGui::Checkbox("Show stats", &m_showStats);
		if (ImGui::Checkbox("Show axes", &m_showAxes)) {
			showAxes(m_showAxes);
		}
		if (ImGui::Checkbox("Show reference plane", &m_showReferencePlane));
	}
	if (ImGui::CollapsingHeader("Simulation Parameters",
		ImGuiTreeNodeFlags_DefaultOpen)) {
		drawSimulationParameterMenu();
	}
	if (ImGui::CollapsingHeader("Recording")) {
		bool hasRecords = p_simulator->getRecords().size() > 0 &&
			p_simulator->getRecords()[0].size() > 0;
		if (!hasRecords) {
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0, 0, 0, 0));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0, 0, 0, 0));
		}
		if (ImGui::Button("Export Recording", ImVec2(-1, 0))) {
			if (hasRecords) {
				exportRecording();
			}
		}
		if (!hasRecords) {
			ImGui::PopStyleColor();
			ImGui::PopStyleColor();
			ImGui::PopStyleColor();
		}
		if (ImGui::InputInt("Steps in Recording to keep", &m_numRecords, 0,
			0)) {
			p_simulator->setNumRecords(m_numRecords);
		}
		bool isRecording = p_simulator->isRecording();
		ImGui::PushStyleColor(ImGuiCol_Button,
			isRecording ? ImVec4(0.98f, 0.26f, 0.26f, 0.40f)
			: ImVec4(0.26f, 0.98f, 0.40f, 0.40f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
			isRecording ? ImVec4(0.98f, 0.26f, 0.26f, 1.0f)
			: ImVec4(0.26f, 0.98f, 0.40f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive,
			isRecording ? ImVec4(0.98f, 0.26f, 0.00f, 0.9f)
			: ImVec4(0.00f, 0.98f, 0.40f, 0.9f));
		if (ImGui::Button(isRecording ? "Stop Recording" : "Start Recording",
			ImVec2(-1, 0))) {
			p_simulator->setRecording(!isRecording);
		}
		ImGui::PopStyleColor();
		ImGui::PopStyleColor();
		ImGui::PopStyleColor();
	}
	return false;
}

void Gui::showAxes(bool show_axes) {
	if (show_axes && m_axesID < 0) {
		Eigen::RowVector3d origin = Eigen::Vector3d::Zero();
		m_axesID = addArrow(origin, Eigen::Vector3d(1, 0, 0),
			Eigen::Vector3d(1, 0, 0));
		addArrow(origin, Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 1, 0));
		addArrow(origin, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));
	}
	if (!show_axes && m_axesID >= 0) {
		removeArrow(m_axesID);
		removeArrow(m_axesID + 1);
		removeArrow(m_axesID + 2);
		m_axesID = -1;
	}
}