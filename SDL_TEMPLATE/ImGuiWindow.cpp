#include <SDL.h>
#include <glad/glad.h>
#include <string>
#include <spdlog/spdlog.h>
#include "ImGuiWindow.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_sdl2.h"
#include "ProgramValues.h"
#include "Model.h"
#include "Camera.h"
#include "Game.h"
#include "GameWindow.h"
#include "CameraPair.h"

#ifdef _WIN32

#include <windows.h>
#include <psapi.h>

static Uint32 lastTime = 0;

void ImGuiWindow::retrieveMemoryUsage() {
	Uint32 now = SDL_GetTicks();

	if (now - lastTime > 1000) {
		PROCESS_MEMORY_COUNTERS_EX pmc;
		if (GetProcessMemoryInfo(GetCurrentProcess(),
								 (PROCESS_MEMORY_COUNTERS*)&pmc,
								 sizeof(pmc))) 
		{
			SIZE_T ramUsed = pmc.WorkingSetSize;
			ProgramValues::GameWindow::RAMusedMB = static_cast<float>(ramUsed) / (1024.0f * 1024.0f);
		}

		lastTime = now;
	}
}

#endif

static Game* game = Game::getInstance();

const static constexpr char* OPENGL_VERSION = "#version 430";

ImGuiWindow::ImGuiWindow() {}

ImGuiWindow* ImGuiWindow::getInstance() {
	static ImGuiWindow instance;
	return &instance;
}

bool ImGuiWindow::init(SDL_Window* window, SDL_GLContext glContext) const {
	spdlog::info("ImGui initializing...");

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows
	//io.ConfigViewportsNoAutoMerge = true;
	//io.ConfigViewportsNoTaskBarIcon = true;

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
	ImGuiStyle& style = ImGui::GetStyle();
	if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
		style.WindowRounding = 0.0f;
		style.Colors[ImGuiCol_WindowBg].w = 1.0f;
	}

	// Setup Platform/Renderer backends
	if (!ImGui_ImplSDL2_InitForOpenGL(window, glContext)) {
		spdlog::error("Failed to initialize ImGui SDL2 backend.");
		return false;
	}
	
	if (!ImGui_ImplOpenGL3_Init(OPENGL_VERSION)) {
		spdlog::error("Failed to initialize ImGui OpenGL3 backend.");
		return false;
	}

	spdlog::info("ImGui initialized successfully.");
	return true;
}

void ImGuiWindow::render() {
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplSDL2_NewFrame();
	ImGui::NewFrame();

	ImGuiIO& io = ImGui::GetIO();
	static ImGuiComboFlags flags = 0;

	ImGui::ShowDemoWindow();


	{
		ImGui::Begin("Window##Window");
		ImGui::Text("Dimension: %d x %d", ProgramValues::GameWindow::width, ProgramValues::GameWindow::height);
		ImGui::Text("Ram Used MB: %f", ProgramValues::GameWindow::RAMusedMB);
		ImGui::Text("Delta Time: %f", ProgramValues::GameWindow::deltaTime);
		ImGui::Text("Measured FPS: %d", ProgramValues::GameWindow::FPS);
		ImGui::Text("FPS Limit: %d", ProgramValues::GameWindow::FPS_LIMIT);
		if (ImGui::Button("60 FPS - Power Saver##Window")) {
			ProgramValues::GameWindow::FPS_LIMIT = 60;
		}
		if (ImGui::Button("120 FPS - Gameplay##Window")) {
			ProgramValues::GameWindow::FPS_LIMIT = 120;
		}
		if (ImGui::Button("240 FPS - Competitive##Window")) {
			ProgramValues::GameWindow::FPS_LIMIT = 240;
		}
		if (ImGui::Button("Unlimited FPS - Unlimited##Window")) {
			ProgramValues::GameWindow::FPS_LIMIT = 99999;
		}
		ImGui::End();
	}
	





	{
		ImGui::Begin("Camera##Cam");

		{

			const char* items[] = { "Freefly", "Camera1"};
			static int item_selected_idx = 0; // Here we store our selection data as an index.

			// Pass in the preview value visible before opening the combo (it could technically be different contents or not pulled from items[])
			const char* combo_preview_value = items[item_selected_idx];

			if (ImGui::BeginCombo("Cameras##Cam", combo_preview_value, flags)) {
				for (int n = 0; n < IM_ARRAYSIZE(items); n++) {
					const bool is_selected = (item_selected_idx == n);
					if (ImGui::Selectable(items[n], is_selected))
						item_selected_idx = n;

					// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}

			switch (item_selected_idx) {
				case 0: ProgramValues::Cameras::cameraReference = &ProgramValues::Cameras::freeFly;		break;
				case 1: ProgramValues::Cameras::cameraReference = &ProgramValues::Cameras::camera1;		break;
				default: break;
			}

			Camera* cameraRef = ProgramValues::Cameras::cameraReference;

			if (cameraRef) {
				ImGui::DragFloat("FOV##Cam", &cameraRef->fov, 0.01f);
				ImGui::DragFloat("Near Clip##Cam", &cameraRef->nearClip, 0.01f, 0.0f, 10000.0f, "%.2f");
				ImGui::DragFloat("Far Clip##Cam", &cameraRef->farClip, 0.01f, 0.0f, 10000.0f, "%.2f");
				ImGui::DragFloat("Position X##Cam", &cameraRef->position.x, 0.01f);
				ImGui::DragFloat("Position Y##Cam", &cameraRef->position.y, 0.01f);
				ImGui::DragFloat("Position Z##Cam", &cameraRef->position.z, 0.01f);
				ImGui::DragFloat("Yaw##Cam", &cameraRef->yaw, 0.1f, -360.0f, 360.0f);
				ImGui::DragFloat("Pitch##Cam", &cameraRef->pitch, 0.1f, -89.0f, 89.0f);

				if (cameraRef->pitch > 89.0f)
					cameraRef->pitch = 89.0f;
				if (cameraRef->pitch < -89.0f)
					cameraRef->pitch = -89.0f;

				ProgramValues::GameWindow::projection = glm::perspective(
					glm::radians(cameraRef->fov),
					(float)game->gameWindow->width() / (float)game->gameWindow->height(),
					cameraRef->nearClip,
					cameraRef->farClip
				);

				cameraRef->updateCameraVectors();

				if (item_selected_idx != 0) {
					Model* cameraModel = CameraPair::getModel(*cameraRef);

					if (cameraModel) {
						cameraModel->translation = cameraRef->position;

						cameraModel->rotateAxis = glm::vec3(0.0f, 1.0f, 0.0f);
						cameraModel->radiansRotate = glm::radians(cameraRef->yaw);

						cameraModel->updateModelMatrix();
					} else {
						item_selected_idx == 0;
					}

				}
			}
		}


		ImGui::End();
	}





	
	{
		ImGui::Begin("Models##Models");

		{
			Model* objectModelRef = nullptr;

			const char* items[] = { "Landscape", "Cube", "Camera1"};
			static int item_selected_idx = 0; // Here we store our selection data as an index.

			// Pass in the preview value visible before opening the combo (it could technically be different contents or not pulled from items[])
			const char* combo_preview_value = items[item_selected_idx];

			if (ImGui::BeginCombo("Models##Models", combo_preview_value, flags)) {
				for (int n = 0; n < IM_ARRAYSIZE(items); n++) {
					const bool is_selected = (item_selected_idx == n);
					if (ImGui::Selectable(items[n], is_selected))
						item_selected_idx = n;

					// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}

			switch (item_selected_idx) {
				case 0: objectModelRef = &ProgramValues::GameObjects::landscape;	break;
				case 1: objectModelRef = &ProgramValues::GameObjects::cube;			break;
				case 2: objectModelRef = &ProgramValues::GameObjects::camera1;		break;
					break;
				default: break;
			}

			if (objectModelRef) {
				ImGui::DragFloat("Scale##Models", &objectModelRef->scale, 0.2f, 0.01f, 100000.0f, "%.2f");
				ImGui::DragFloat("Translate X##Models", &objectModelRef->translation.x, 0.05);
				ImGui::DragFloat("Translate Y##Models", &objectModelRef->translation.y, 0.05);
				ImGui::DragFloat("Translate Z##Models", &objectModelRef->translation.z, 0.05);
				ImGui::DragFloat("Radians##Models", &objectModelRef->radiansRotate, 0.2f, 0.01f, 100000.0f, "%.2f");
				ImGui::DragFloat("Rotate X##Models", &objectModelRef->rotateAxis.x, 0.05);
				ImGui::DragFloat("Rotate Y##Models", &objectModelRef->rotateAxis.y, 0.05);
				ImGui::DragFloat("Rotate Z##Models", &objectModelRef->rotateAxis.z, 0.05);

				objectModelRef->updateModelMatrix();
			}
		}

		ImGui::End();
	}






	{
		ImGui::Begin("Lights");

		ImGui::Text("Directional Lights##Dir");

		{
			ProgramValues::DirLight* dirLightRef = nullptr;

			const char* items[] = { "DR Light1" };
			static int item_selected_idx = 0; // Here we store our selection data as an index.

			// Pass in the preview value visible before opening the combo (it could technically be different contents or not pulled from items[])
			const char* combo_preview_value = items[item_selected_idx];

			if (ImGui::BeginCombo("Directional Lights##Dir", combo_preview_value, flags)) {
				for (int n = 0; n < IM_ARRAYSIZE(items); n++) {
					const bool is_selected = (item_selected_idx == n);
					if (ImGui::Selectable(items[n], is_selected))
						item_selected_idx = n;

					// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}

			switch (item_selected_idx) {
				case 0: dirLightRef = &ProgramValues::Lights::dr_sun;		break;
				default: break;
			}

			if (dirLightRef) {
				float ambient = dirLightRef->ambient.x;
				float diffuse = dirLightRef->diffuse.x;
				float specular = dirLightRef->specular.x;

				ImGui::DragFloat("X##Dir", &dirLightRef->direction.x, 0.01f, -1.0f, 1.0f, "%.2f");
				ImGui::DragFloat("Y##Dir", &dirLightRef->direction.y, 0.01f, -1.0f, 1.0f, "%.2f");
				ImGui::DragFloat("Z##Dir", &dirLightRef->direction.z, 0.01f, -1.0f, 1.0f, "%.2f");
				ImGui::DragFloat("Ambient##Dir", &ambient, 0.01f, 0.0f, 1.0f, "%.2f");
				ImGui::DragFloat("Diffuse##Dir", &diffuse, 0.01f, 0.0f, 1.0f, "%.2f");
				ImGui::DragFloat("Specular##Dir", &specular, 0.01f, 0.0f, 1.0f, "%.2f");

				dirLightRef->ambient = glm::vec3(ambient);
				dirLightRef->diffuse = glm::vec3(diffuse);
				dirLightRef->specular = glm::vec3(specular);
			}
		}






		ImGui::Text("Point Lights");
		
		





		ImGui::Text("Spot Lights##Spot");
		{
			ProgramValues::SpotLight* spotLightRef = nullptr;

			const char* items[] = { "SPT Light1" };
			static int item_selected_idx = 0; // Here we store our selection data as an index.

			// Pass in the preview value visible before opening the combo (it could technically be different contents or not pulled from items[])
			const char* combo_preview_value = items[item_selected_idx];

			if (ImGui::BeginCombo("Spot Lights##Spot", combo_preview_value, flags)) {
				for (int n = 0; n < IM_ARRAYSIZE(items); n++) {
					const bool is_selected = (item_selected_idx == n);
					if (ImGui::Selectable(items[n], is_selected))
						item_selected_idx = n;

					// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
					if (is_selected)
						ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}

			switch (item_selected_idx) {
				case 0: spotLightRef = &ProgramValues::Lights::spt_pov;		break;
				default: break;
			}

			if (spotLightRef) {
				float ambient = spotLightRef->ambient.x;
				float diffuse = spotLightRef->diffuse.x;
				float specular = spotLightRef->specular.x;

				ImGui::DragFloat("Inner Cutoff##Spot", &spotLightRef->innerCutoff, 0.01f, 0.0f, 180.0f, "%.2f");
				ImGui::DragFloat("Outer Cutoff##Spot", &spotLightRef->outerCutoff, 0.01f, 0.0f, 180.0f, "%.2f");
				ImGui::DragFloat("Constant##Spot", &spotLightRef->constant, 0.01f, 0.000001f, 1.0f, "%.5f");
				ImGui::DragFloat("Linear##Spot", &spotLightRef->linear, 0.001f, 0.000001f, 1.0f, "%.5f");
				ImGui::DragFloat("Quadratic##Spot", &spotLightRef->quadratic, 0.0001f, 0.000001f, 1.0f, "%.5f");
				ImGui::DragFloat("Ambient##Spot", &ambient, 0.01f, 0.0f, 1.0f, "%.3f");
				ImGui::DragFloat("Diffuse##Spot", &diffuse, 0.01f, 0.0f, 1.0f, "%.3f");
				ImGui::DragFloat("Specular##Spot", &specular, 0.01f, 0.0f, 1.0f, "%.3f");


				spotLightRef->ambient = glm::vec3(ambient);
				spotLightRef->diffuse = glm::vec3(diffuse);
				spotLightRef->specular = glm::vec3(specular);
			}
		}

		ImGui::End();
	}

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
		SDL_Window* backup_current_window = SDL_GL_GetCurrentWindow();
		SDL_GLContext backup_current_context = SDL_GL_GetCurrentContext();
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
		SDL_GL_MakeCurrent(backup_current_window, backup_current_context);
	}
}

void ImGuiWindow::clean() {
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();
}