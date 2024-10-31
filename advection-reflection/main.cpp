#include <complex>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>

const int RES_X = 128;
const int RES_Y = 128;
const int RES_Z = 128;
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
    FluidSim(): m_density(RES_X * RES_Y * RES_Z, 0.0), m_velocity(RES_X * RES_Y * RES_Z, Eigen::Vector3f::Zero()) {
    }

    void create_sphere_density() {
        for (int x = 0; x < RES_X; x++) {
            for (int y = 0; y < RES_Y; y++) {
                for (int z = 0; z < RES_Z; z++) {
                    int index = x + y * RES_X + z * RES_X * RES_Y;

                    int xx = x - RES_X / 2;
                    int yy = y - RES_Y / 2;
                    int zz = z - RES_Z / 2;
                    if (xx * xx + yy * yy + zz * zz < 32 * 32) {
                        m_density[index] = float(32 * 32 - (xx * xx + yy * yy + zz * zz)) / float(32 * 32) * 0.4;
                    } else {
                        m_density[index] = 0.0;
                    }
                }
            }
        }
    }

    void create_circular_flow() {
        Eigen::Vector3f center(RES_X / 2.0f, RES_Y / 2.0f, RES_Z / 2.0f); // Center of vortex
        float strength = 1.0f; // Controls the intensity of the rotation

        for (int x = 0; x < RES_X; x++) {
            for (int y = 0; y < RES_Y; y++) {
                for (int z = 0; z < RES_Z; z++) {
                    Eigen::Vector3f pos(x, y, z);
                    Eigen::Vector3f direction = pos - center;
                    direction = Eigen::Vector3f(-direction.y(), direction.x(), 0.0f); // Rotate around z-axis
                    direction.normalize();
                    m_velocity[index(x, y, z)] = direction * strength / (pos - center).norm(); // Decay with distance
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

    void update() {

        float m_step = 0.01f;

        // Great video https://www.youtube.com/watch?v=iKAVRgIrUOU
        // 1. Modify velocity values (e.g. gravity)
        // 2. make fluid incompressable (projection)
        // 3. Move the velocity field (advection)

        mac_cormack_advection(m_step / 2.0f);
        projection();
        reflection();
        mac_cormack_advection(m_step / 2.0f);
        projection();
    }

private:

    // Should be represeneted as a staggered grid. velocity vectors are not in the center of the cell
    // https://www.researchgate.net/profile/Ke-Qing-Xia/publication/325053508/figure/fig1/AS:625094766370816@1526045614156/Schematic-of-the-staggered-grid_W640.jpg
    std::vector<Eigen::Vector3f> m_velocity;
    std::vector<float> m_density;

    float m_advection_speed = 1.0f;

    static size_t index(int x, int y, int z) {
        return std::min(std::max(0, x + y * RES_X + z * RES_X * RES_Y), RES_X*RES_Y*RES_Z - 1);
    }

    void mac_cormack_advection(float t_step) {

        // create backup
        std::vector<float> d(m_density);
        std::vector<Eigen::Vector3f> v(m_velocity);

        // predictor step
        for(int x = 1; x < RES_X - 1; x++) {
            for(int y = 1; y < RES_Y - 1; y++) {
                for(int z = 1; z < RES_Z - 1; z++) {
                    size_t i = index(x, y, z);
                    Eigen::Vector3f dx = v[index(x + 1, y, z)] - v[i]; // Spacing is one?
                    Eigen::Vector3f dy = v[index(x, y + 1, z)] - v[i]; // Spacing is one?
                    Eigen::Vector3f dz = v[index(x, y, z + 1)] - v[i]; // Spacing is one?
                    m_velocity[i] = v[i] - t_step * (dx + dy + dz);
                }
            }
        }

        // Corrector Step
        for(int x = 1; x < RES_X - 1; x++) {
            for(int y = 1; y < RES_Y - 1; y++) {
                for(int z = 1; z < RES_Z - 1; z++) {
                    size_t i = index(x, y, z);
                    Eigen::Vector3f dx = m_velocity[i] - m_velocity[index(x - 1, y, z)]; // Spacing is one?
                    Eigen::Vector3f dy = m_velocity[i] - m_velocity[index(x, y - 1, z)]; // Spacing is one?
                    Eigen::Vector3f dz = m_velocity[i] - m_velocity[index(x, y, z - 1)]; // Spacing is one?
                    m_velocity[i] = (m_velocity[i] + v[i]) / 2.0f - t_step / 2.0f * (dx + dy + dz);
                }
            }
        }

        // Clamping
        for(int x = 1; x < RES_X - 1; x++) {
            for(int y = 1; y < RES_Y - 1; y++) {
                for(int z = 1; z < RES_Z - 1; z++) {
                    size_t i = index(x, y, z);

                    Eigen::Vector3f min_val = v[i];
                    Eigen::Vector3f max_val = v[i];
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dz = -1; dz <= 1; dz++) {
                                size_t neighbor_index = index(x + dx, y + dy, z + dz);
                                min_val = min_val.cwiseMin(v[neighbor_index]);
                                max_val = max_val.cwiseMax(v[neighbor_index]);
                            }
                        }
                    }

                    m_velocity[i] = m_velocity[i].cwiseMax(min_val).cwiseMin(max_val);
                }
            }
        }


        for(int x = 1; x < RES_X - 1; x++) {
            for(int y = 1; y < RES_Y - 1; y++) {
                for(int z = 1; z < RES_Z - 1; z++) {
                    Eigen::Vector3f vel = m_velocity[index(x, y, z)];
                    m_density[index(x, y, z)] = d[index(x - vel.x(), y - vel.y(), z - vel.z())];
                }
            }
        }


    }

    void projection() {

    }

    void reflection() {

    }

};

int main() {
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
    sim.create_circular_flow();

    // Render loop
    while (!glfwWindowShouldClose(window)) {
        // Input
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        sim.update();

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
