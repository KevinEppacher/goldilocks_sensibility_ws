#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> 
#include <iostream>
#include <string>
#include <regex>

//ROS and Moveit
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>




#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif
   

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}


// Main code
int main(int argc, char** argv)
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(700, 700, "Example HMI for Robotermodellierung", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwSetWindowSizeLimits(window, 700, 700, 700, 700);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    ImVec4 clear_color = ImVec4(0.40f, 0.50f, 0.60f, 0.45f);
    
    
    //ROS Stuff
    
    ros::init(argc, argv, "hmi");
    ros::NodeHandle nh;
    int speed = 1;

   
    ///////////////
    // Main loop //    
    ///////////////
    
    while (!glfwWindowShouldClose(window))

    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();


        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        
			ImGui::SetNextWindowSize(ImVec2(600,600));
			ImGui::SetNextWindowPos(ImVec2(50,50));
			
			//Start of the Window
            ImGui::Begin("Example HMI" , NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | 4 | 256 | 1);
            // Buttons return true when clicked (most widgets return true when edited/activated)

            
            //Close Button
            ImGui::SetCursorPos(ImVec2(500,500));
            if(ImGui::Button("CLOSE"))
            {
                    glfwWindowShouldClose(window);
                    break;
            }
            
            //X+ Button
            ImGui::SetCursorPos(ImVec2(275,50));
			if(ImGui::Button("X+", ImVec2(50.0,50.0)))
			{       
                std::cout<<"X+ Button Pressed"<<std::endl;
			}
			
			//Y- Button
			ImGui::SetCursorPos(ImVec2(200,125)); 
         	if(ImGui::Button("Y-", ImVec2(50.0,50.0)))
         	{
         		std::cout<<"Y- Button Pressed"<<std::endl;
         	}
         	                    
            
            //Y+ Button
            ImGui::SameLine();
            ImGui::SetCursorPos(ImVec2(350,125)); 
            if(ImGui::Button("Y+", ImVec2(50.0,50.0)))
            {
                std::cout<<"Y+ Button Pressed"<<std::endl;
            }
            
            //X- Button
            ImGui::SetCursorPos(ImVec2(275,200));
            if(ImGui::Button("X-", ImVec2(50.0,50.0)))
            {
                std::cout<<"X- Button Pressed"<<std::endl;
            }
            
            //Z+ Button
            ImGui::SetCursorPos(ImVec2(425,75));
            if(ImGui::Button("Z+", ImVec2(50,50)))
            {
                std::cout<<"Z+ Button Pressed"<<std::endl;
            }
            
            //Z- Button
            ImGui::SetCursorPos(ImVec2(425,175));
            if(ImGui::Button("Z-", ImVec2(50,50)))
            {
                std::cout<<"Z- Button Pressed"<<std::endl;
            }
            
            
            //Stepsize Slider
            ImGui::SetCursorPos(ImVec2(50,350));
            ImGui::SliderInt("Steppsize(%)",&speed, 1,100);
            
			//Framerate for HMI Window 
			ImGui::SetCursorPos(ImVec2(5,5));
            ImGui::Text("%.2f",ImGui::GetIO().Framerate);
            
             
            ImGui::End();
            //End of the Window



        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        
    }


    // Cleanup
    ros::shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
