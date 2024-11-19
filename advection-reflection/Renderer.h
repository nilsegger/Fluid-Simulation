//
// Created by nils on 12/11/24.
//

#ifndef RENDERER_H
#define RENDERER_H

#include <complex>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>

class Line {
    int shaderProgram;
    unsigned int VBO, VAO;
    std::vector<float> vertices;
    Eigen::Vector3f startPoint;
    Eigen::Vector3f endPoint;
    Eigen::Matrix4f MVP;
    Eigen::Vector3f lineColor;

public:
    Line(Eigen::Vector3f start, Eigen::Vector3f end) {
        startPoint = start;
        endPoint = end;
        lineColor = Eigen::Vector3f(0.0f, 0.0f, 1.0f); // vec3(1,1,1);
        //MVP = mat4(1.0f);

        const char *vertexShaderSource = "#version 330 core\n"
                "layout (location = 0) in vec3 aPos;\n"
                "uniform mat4 MVP;\n"
                "void main()\n"
                "{\n"
                "   gl_Position = MVP * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
                "}\0";
        const char *fragmentShaderSource = "#version 330 core\n"
                "out vec4 FragColor;\n"
                "uniform vec3 color;\n"
                "void main()\n"
                "{\n"
                "   FragColor = vec4(color, 1.0f);\n"
                "}\n\0";

        // vertex shader
        int vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);
        // check for shader compile errors

        // fragment shader
        int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);
        // check for shader compile errors

        // link shaders
        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);
        // check for linking errors

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        vertices = {
            start.x(), start.y(), start.z(),
            end.x(), end.y(), end.z(),
        };

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *) 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    void draw(Eigen::Matrix4Xf mvp, Eigen::Vector3f color) {
        glUseProgram(shaderProgram);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "MVP"), 1, GL_FALSE, mvp.data());
        glUniform3fv(glGetUniformLocation(shaderProgram, "color"), 1, color.data());

        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES, 0, 2);
    }

    ~Line() {
        // TODO: this leads to seg fault?
        // glDeleteVertexArrays(1, &VAO);
        // glDeleteBuffers(1, &VBO);
        // glDeleteProgram(shaderProgram);
    }
};

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    glViewport(0, 0, width, height);
}

class Renderer {
public:
    int open(const int width, const int height) {
        // Initialize GLFW
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return -1;
        }

        // Create a GLFW window
        window = glfwCreateWindow(width, height, "Quad Example", nullptr, nullptr);
        if (!window) {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return -1;
        }
        glfwMakeContextCurrent(window);
        glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

        // Load GLAD
        if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
            std::cerr << "Failed to initialize GLAD" << std::endl;
            return -1;
        }

        // Build and compile the shader program
        GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
        glCompileShader(vertexShader);

        GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
        glCompileShader(fragmentShader);

        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);

        // Delete the shaders as they're linked into the program now
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        // Set up vertex data and buffers
        float vertices[] = {
            -1.0f, -1.0f, // Bottom left
            1.0f, -1.0f, // Bottom right
            1.0f, 1.0f, // Top right
            -1.0f, 1.0f // Top left
        };

        unsigned int indices[] = {
            0, 1, 2, // First triangle
            0, 2, 3 // Second triangle
        };

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *) 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
        glBindVertexArray(0); // Unbind VAO

        return 0;
    }

    void close() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glDeleteProgram(shaderProgram);

        glfwDestroyWindow(window);
        glfwTerminate();
    }

    void clear_frame() {
        // Input
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        // Render
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
    }

    void bind_fluid_shader_data(const int width, const int height, const int resx, const int resy, const int resz, const std::vector<float>& data, float depth) {
        GLint windowLocation = glGetUniformLocation(shaderProgram, "window");
        glUniform2f(windowLocation, width, height);

        glUniform3f(glGetUniformLocation(shaderProgram, "resolution"), resx, resy, resz);

        glUniform1f(glGetUniformLocation(shaderProgram, "depth"), depth);

        GLuint textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_3D, textureID);

        // Set texture parameters
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

        // Upload data to the 3D texture
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, resx, resy, resz, 0, GL_RED, GL_FLOAT,
                     data.data());

        glBindTexture(GL_TEXTURE_3D, 0);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, textureID);
        // Set the sampler3D uniform to use texture unit 0
        glUniform1i(glGetUniformLocation(shaderProgram, "densityTexture"), 0);

        // Draw the quad
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    }

    void draw() {
        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    bool windowHasBeenClosed() {
        return glfwWindowShouldClose(window);
    }

    bool checkSpaceKeyReleased() {
        int currentSpaceState = glfwGetKey(window, GLFW_KEY_SPACE);
        bool released = (wasSpacePressed && currentSpaceState == GLFW_RELEASE);
        wasSpacePressed = (currentSpaceState == GLFW_PRESS);
        return released;
    }

    bool checkKey(const int key, bool& boolean) {
        int currentSpaceState = glfwGetKey(window, key);
        bool released = (boolean && currentSpaceState == GLFW_RELEASE);
        boolean = (currentSpaceState == GLFW_PRESS);
        return released;
    }

    bool upPressed() {
        return checkKey(GLFW_KEY_UP, keyUpPressed);
    }

    bool downPressed() {
        return checkKey(GLFW_KEY_DOWN, keyDownPressed);
    }

    bool pPressed() {
        return checkKey(GLFW_KEY_P, keypPressed);
    }

    bool sPressed() {
        return checkKey(GLFW_KEY_S, keysPressed);
    }

    bool rPressed() {
        return checkKey(GLFW_KEY_R, keysRPressed);
    }

    bool Pressed1() {
        return checkKey(GLFW_KEY_1, keys1Pressed);
    }

    bool Pressed2() {
        return checkKey(GLFW_KEY_2, keys2Pressed);
    }

    bool Pressed3() {
        return checkKey(GLFW_KEY_3, keys3Pressed);
    }

private:

    GLFWwindow *window = nullptr;
    GLuint VAO, VBO, EBO, shaderProgram;

    bool wasSpacePressed = false;
    bool keyUpPressed = false;
    bool keysRPressed = false;
    bool keyDownPressed = false;
    bool keypPressed = false;
    bool keysPressed = false;
    bool keys1Pressed = false;
    bool keys2Pressed = false;
    bool keys3Pressed = false;

    // Vertex shader source code
    const char *vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec2 aPos;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

    // Fragment shader source code
    const char *fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 resolution;
uniform vec2 window;
uniform float depth;

uniform sampler3D densityTexture;

void main() {

    vec2 scale = window / resolution.xy;

    // Ray Marching into depth. This will only work because of the setup of the scene.
    // There is no rotation and no perspective... Hence xy coords can be scaled to resolution xy and z as depth is simply res z
    // Can probably still be optimised a lot by increasing step sizes, when nothing is hit

    float maxDensity = 0.0f;
    vec4 outputColor = vec4(0.0);
    float step = 1.0f;
    float maxStep = 16.0f;
    for(float t = depth; t <= resolution.z; t += step) {

        vec3 uv = vec3(gl_FragCoord.x / scale.x, gl_FragCoord.y / scale.y, t) / resolution;
        float pointDensity = texture(densityTexture, uv).r;

        // Somehow made the rendering slower?
        if (pointDensity < 0) {
            step = min(step * 2.0f, maxStep);  // Limit the step size growth
        } else {
            step = 1.0f;
        }

        vec4 sampleColor = vec4(vec3(0.5), pointDensity);

        vec3 updatedColor = outputColor.rgb +  sampleColor.rgb * sampleColor.a * (1.0 -  outputColor.a);
        float updatedAlpha = outputColor.a + sampleColor.a * (1.0 - outputColor.a);
        outputColor = vec4(updatedColor, updatedAlpha);

        if (outputColor.a > 0.99) break;
    }

    FragColor = outputColor; //vec4(maxDensity, maxDensity, maxDensity, 1.0); // RGB based on coordinates
}
)";
};

#endif //RENDERER_H
