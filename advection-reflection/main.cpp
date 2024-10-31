#include <complex>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <chrono>

const int RES_X = 64;
const int RES_Y = 64;
const int RES_Z = 64;
const int WIDTH = 768;
const int HEIGHT = 768;


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

uniform sampler3D densityTexture;

void main() {

    vec2 scale = window / resolution.xy;

    // Ray Marching into depth. This will only work because of the setup of the scene.
    // There is no rotation and no perspective... Hence xy coords can be scaled to resolution xy and z as depth is simply res z
    // Can probably still be optimised a lot by increasing step sizes, when nothing is hit

    float maxDensity = 0.0f;
    vec4 outputColor = vec4(0.0);
    for(float t = 0.0; t <= resolution.z; t += 1.0) {

        vec3 uv = vec3(gl_FragCoord.x / scale.x, gl_FragCoord.y / scale.y, t) / resolution;
        float pointDensity = texture(densityTexture, uv).r;

        vec4 sampleColor = vec4(vec3(0.5), pointDensity);

        vec3 updatedColor = outputColor.rgb +  sampleColor.rgb * sampleColor.a * (1.0 -  outputColor.a);
        float updatedAlpha = outputColor.a + sampleColor.a * (1.0 - outputColor.a);
        outputColor = vec4(updatedColor, updatedAlpha);

        if (outputColor.a > 0.99) break;
    }

    FragColor = outputColor; //vec4(maxDensity, maxDensity, maxDensity, 1.0); // RGB based on coordinates
}
)";

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    glViewport(0, 0, width, height);
}

class FluidSim {
public:

    FluidSim(): m_vu(RES_X * RES_Y * RES_Z, 0.0f), m_vv(RES_X * RES_Y * RES_Z, 0.0f),
                m_vw(RES_X * RES_Y * RES_Z, 0.0f), m_s(RES_X*RES_Y*RES_Z, 1.0f),
                m_density(RES_X * RES_Y * RES_Z, 0.0) {
    }

    void create_sphere_density() {
        for (int x = 0; x < RES_X; x++) {
            for (int y = 0; y < RES_Y; y++) {
                for (int z = 0; z < RES_Z; z++) {
                    int index = x + y * RES_X + z * RES_X * RES_Y;

                    int xx = x - RES_X / 2;
                    int yy = y - RES_Y / 2;
                    int zz = z - RES_Z / 2;
                    if (xx * xx + yy * yy + zz * zz < 16 * 16) {
                        m_density[index] = float(16 * 16 - (xx * xx + yy * yy + zz * zz)) / float(16 * 16) * 0.1;
                    } else {
                        m_density[index] = 0.0;
                    }
                }
            }
        }
    }

    void bind_density(GLuint shaderProgram) {
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
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, RES_X, RES_Y, RES_Z, 0, GL_RED, GL_FLOAT,
                     m_density.data());

        glBindTexture(GL_TEXTURE_3D, 0);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, textureID);
        // Set the sampler3D uniform to use texture unit 0
        glUniform1i(glGetUniformLocation(shaderProgram, "densityTexture"), 0);
    }

    void update(float t_step) {

        // Great video https://www.youtube.com/watch?v=iKAVRgIrUOU
        // 1. Modify velocity values (e.g. gravity)

        iterate([this, t_step](int x, int y, int z) {
            m_vv[index(x, y, z)] = m_vv[index(x, y, z)] + (t_step * -9.81f);
        });

        std::vector ss_vv(m_vv);
        std::vector ss_vu(m_vu);
        std::vector ss_vw(m_vw);
        std::vector ss_d(m_density);

        // 2. make fluid incompressable (projection)
        iterate([this, t_step, &ss_vv, &ss_vu, &ss_vw](int x, int y, int z) {

            int i = index(x, y, z);
            int ixpp = index(x+1, y, z);
            int ixmm = index(x-1, y, z);
            int iypp = index(x, y+1, z);
            int iymm = index(x, y-1, z);
            int izpp = index(x, y, z+1);
            int izmm = index(x, y, z-1);

            // Gauss-Seidel method
            float divergence = 1.0f * (m_vu[i] - m_vu[ixpp] + m_vv[i] - m_vv[iypp] + m_vw[i] - m_vw[izpp]);
            float s = m_s[ixpp] + m_s[ixmm] + m_s[iypp]+ m_s[iymm]+ m_s[izpp] + m_s[izmm];
            ss_vu[i] += divergence * (m_s[i]) / s;
            ss_vu[ixpp] -= divergence * (m_s[ixpp]) / s;
            ss_vv[i] += divergence * (m_s[i]) / s;
            ss_vv[iypp] -= divergence * (m_s[iypp]) / s;
            ss_vw[i] += divergence * (m_s[i]) / s;
            ss_vw[izpp] -= divergence * (m_s[izpp]) / s;
        });

        // 3. Move the velocity field (advection)
        iterate([this, t_step, &ss_vv, &ss_vu, &ss_vw, &ss_d](int x, int y, int z) {
            // semi-lagrangian

            // TODO: check for obstacles
            int i = index(x, y, z);

            // Backtrack, find out where "particles" come from
            // u component
            m_vu[i] = sample(ss_vu, ss_vv, ss_vw, x, y, z, SampleDirection::X, t_step);
            m_vv[i] = sample(ss_vu, ss_vv, ss_vw, x, y, z, SampleDirection::Y, t_step);
            m_vw[i] = sample(ss_vu, ss_vv, ss_vw, x, y, z, SampleDirection::Z, t_step);

            m_density[i] = sample_density(ss_d, ss_vu, ss_vv, ss_vw, x, y, z, t_step);

        });
    }

private:
    // Should be represeneted as a staggered grid. velocity vectors are not in the center of the cell
    // https://www.researchgate.net/profile/Ke-Qing-Xia/publication/325053508/figure/fig1/AS:625094766370816@1526045614156/Schematic-of-the-staggered-grid_W640.jpg
    std::vector<float> m_vu;
    std::vector<float> m_vv;
    std::vector<float> m_vw;

    // 1 means fluid cell, 0 means obstable
    std::vector<float> m_s;

    std::vector<float> m_density;

    float m_advection_speed = 1.0f;

    // staggered grid index
    static int index(int x, int y, int z) {
        return std::min(std::max(0, x + y * (RES_X) + z * (RES_X) * (RES_Y)),
                        (RES_X) * (RES_Y) * (RES_Z) - 1);
    }

    template<typename F>
    void iterate(const F &func) {
        for (int x = 1; x < RES_X - 1; x++) {
            for (int y = 1; y < RES_Y - 1; y++) {
                for (int z = 1; z < RES_Z - 1; z++) {
                    func(x, y, z);
                }
            }
        }
    }

    enum SampleDirection {
        X, Y, Z
    };

    // Move back by new velocity and uniformly sample value
    float sample(std::vector<float>& ss_vu, std::vector<float>& ss_vv, std::vector<float>& ss_vw, int x, int y, int z, SampleDirection direction, float t_step) {
        int i = index(x, y, z);

        // Vector to move back by, different depending on sample direction
        float vx = ss_vu[i] * t_step;
        float vy = ss_vv[i] * t_step;
        float vz = ss_vw[i] * t_step;

        switch(direction) {
            case X: {
                vy = (ss_vv[i] + ss_vv[index(x+1, y, z)] + ss_vv[index(x, y+1, z)] + ss_vv[index(x+1, y+1, z)]) / 4.0f;
                vz = (ss_vw[i] + ss_vw[index(x+1, y, z)] + ss_vw[index(x, y, z+1)] + ss_vw[index(x+1, y, z+1)]) / 4.0f;
                break;
            }
            case Y: {
                vx = (ss_vu[i] + ss_vu[index(x+1, y, z)] + ss_vu[index(x, y+1, z)] + ss_vu[index(x+1, y+1, z)]) / 4.0f;
                vz = (ss_vw[i] + ss_vw[index(x, y+1, z)] + ss_vw[index(x, y, z+1)] + ss_vw[index(x, y+1, z+1)]) / 4.0f;
                break;
            }
            case Z: {
                vx = (ss_vu[i] + ss_vu[index(x+1, y, z)] + ss_vu[index(x, y, z+1)] + ss_vu[index(x+1, y, z+1)]) / 4.0f;
                vy = (ss_vw[i] + ss_vw[index(x, y+1, z)] + ss_vw[index(x, y, z+1)] + ss_vw[index(x, y+1, z+1)]) / 4.0f;
                break;
            }
        }

        // Move back
        float nux = (float)x - vx;
        float nvx = (float)y - vy;
        float nwx = (float)z - vz;

        switch (direction) {
            case X: {
                float w0 = nux - std::floor(nux);
                float w1 = 1.0f - w0;
                return w0 * ss_vu[index(nux, nvx, nwx)] + w1 * ss_vu[index(nux + 1, nvx, nwx)];
            }
            case Y: {
                float w0 = nvx - std::floor(nvx);
                float w1 = 1.0f - w0;
                return w0 * ss_vu[index(nux, nvx, nwx)] + w1 * ss_vu[index(nux, nvx+1, nwx)];
            }
            case Z: {
                float w0 = nwx - std::floor(nwx);
                float w1 = 1.0f - w0;
                return w0 * ss_vu[index(nux, nvx, nwx)] + w1 * ss_vu[index(nux, nvx, nwx+1)];
            }
        }

        return 0.0f;
    }

        // Move back by new velocity and uniformly sample value
    float sample_density(std::vector<float>& density, std::vector<float>& ss_vu, std::vector<float>& ss_vv, std::vector<float>& ss_vw, int x, int y, int z, float t_step) {
        int i = index(x, y, z);

        // Vector to move back by, different depending on sample direction
        float vx = (ss_vu[i] + ss_vu[index(x+1, y, z)]) / 2.0f * t_step;
        float vy = (ss_vv[i] + ss_vv[index(x, y+1, z)]) / 2.0f * t_step;
        float vz = (ss_vw[i] + ss_vw[index(x, y, z+1)]) / 2.0f * t_step;

        // Move back, lands within 4 cells origins -> interpolate
        float nx = static_cast<float>(x) - vx;
        float ny = static_cast<float>(y) - vy;
        float nz = static_cast<float>(z) - vz;

        float x1 = std::floor(nx);
        float x2 = nx - x1 >= 0.5 ? std::ceil(nx) : x1 - 1;
        float y1 = std::floor(ny);
        float y2 = ny - y1 >= 0.5 ? std::ceil(ny) : y1 - 1;
        float z1 = std::floor(nz);
        float z2 = ny - z1 >= 0.5 ? std::ceil(nz) : z1 - 1;

        if(x1 > x2) std::swap(x1, x2);
        if(y1 > y2) std::swap(y1, y2);
        if(z1 > z2) std::swap(z1, z2);
        if(x1 == x2) x2++;
        if(y1 == y2) y2++;
        if(z1 == z2) z2++;

        // Trilinier interpolation
        float u = (nx - x1) / (x2 - x1);
        float v = (ny - y1) / (y2 - y1);
        float w = (nz - z1) / (z2 - z1);

        int i1 = index(x1, y1, z1); float w1 = (1.0f - u) * (1.0f - v) * (1.0f - w);
        int i2 = index(x2, y1, z1); float w2 = u * (1.0f - v) * (1.0f - w);
        int i3 = index(x1, y2, z1); float w3 = (1.0f - u) * v * (1.0f - w);
        int i4 = index(x1, y1, z2); float w4 = (1.0f - u) * (1.0f - v) * w;
        int i5 = index(x2, y2, z1); float w5 = u * v * (1.0f - w);
        int i6 = index(x2, y1, z2); float w6 = u * (1.0f - v) * w;
        int i7 = index(x1, y2, z2); float w7 = (1.0f - u) * v * w;
        int i8 = index(x2, y2, z2); float w8 = u*v*w;

        return w1 * density[i1] + w2 * density[i2] + w3 * density[i3] + w4 * density[i4] + w5 * density[i5] + w6 * density[i6] + w7 * density[i7] + w8 * density[i8];
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create a GLFW window
    GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "Quad Example", nullptr, nullptr);
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

    GLuint shaderProgram = glCreateProgram();
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

    GLuint VAO, VBO, EBO;
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

    FluidSim sim;
    sim.create_sphere_density();

    double deltaTime = 0.0;

    // Render loop
    while (!glfwWindowShouldClose(window)) {
        auto startTime = std::chrono::high_resolution_clock::now();
        // Input
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);


        sim.update(std::min(1.0 / 30.0f, deltaTime / 1000.0f));

        // Render
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        GLint windowLocation = glGetUniformLocation(shaderProgram, "window");
        glUniform2f(windowLocation, WIDTH, HEIGHT);

        glUniform3f(glGetUniformLocation(shaderProgram, "resolution"), RES_X, RES_Y, RES_Z);

        sim.bind_density(shaderProgram);

        // Draw the quad
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = endTime - startTime;
        deltaTime = elapsed.count();

        // Print or log the frame time in milliseconds
        std::cout << "Frame time: " << deltaTime << " ms. Using " << std::min(1.0 / 30.0f, deltaTime / 1000.0f) << "s" << std::endl;

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Clean up
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
