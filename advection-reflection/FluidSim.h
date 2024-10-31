#include <igl/edges.h>
#include "Simulation.h"
#include "Grid2.h"
#include "MACGrid2.h"

using namespace std;

/*
 * Simulation of a simple smoke plume rising.
 */
class FluidSim : public Simulation {
public:
    FluidSim() : Simulation(), m_renderV(8, 3), m_renderF(12, 3), m_renderC(12, 3), m_density(128 * 128 * 128),
                 m_density_width(128), m_density_height(128), m_density_depth(128) {
        m_renderV <<
                -0.5, -0.5, -0.5, // Vertex 0
                0.5, -0.5, -0.5, // Vertex 1
                0.5, 0.5, -0.5, // Vertex 2
                -0.5, 0.5, -0.5, // Vertex 3
                -0.5, -0.5, 0.5, // Vertex 4
                0.5, -0.5, 0.5, // Vertex 5
                0.5, 0.5, 0.5, // Vertex 6
                -0.5, 0.5, 0.5; // Vertex 7

        // Define face indices for the 12 triangles (6 faces, 2 triangles per face)
        m_renderF <<
                0, 1, 2, // Triangle 1 of Face 1 (front)
                0, 2, 3, // Triangle 2 of Face 1 (front)
                4, 5, 6, // Triangle 1 of Face 2 (back)
                4, 6, 7, // Triangle 2 of Face 2 (back)
                0, 1, 5, // Triangle 1 of Face 3 (bottom)
                0, 5, 4, // Triangle 2 of Face 3 (bottom)
                2, 3, 7, // Triangle 1 of Face 4 (top)
                2, 7, 6, // Triangle 2 of Face 4 (top)
                0, 3, 7, // Triangle 1 of Face 5 (left)
                0, 7, 4, // Triangle 2 of Face 5 (left)
                1, 2, 6, // Triangle 1 of Face 6 (right)
                1, 6, 5; // Triangle 2 of Face 6 (right)

        // Define colors for each triangle (optional)
        m_renderC <<
                1.0, 0.0, 0.0, // Color for Triangle 1 of Face 1 (Red)
                1.0, 0.0, 0.0, // Color for Triangle 2 of Face 1 (Red)
                0.0, 1.0, 0.0, // Color for Triangle 1 of Face 2 (Green)
                0.0, 1.0, 0.0, // Color for Triangle 2 of Face 2 (Green)
                0.0, 0.0, 1.0, // Color for Triangle 1 of Face 3 (Blue)
                0.0, 0.0, 1.0, // Color for Triangle 2 of Face 3 (Blue)
                1.0, 1.0, 0.0, // Color for Triangle 1 of Face 4 (Yellow)
                1.0, 1.0, 0.0, // Color for Triangle 2 of Face 4 (Yellow)
                1.0, 0.5, 0.0, // Color for Triangle 1 of Face 5 (Orange)
                1.0, 0.5, 0.0, // Color for Triangle 2 of Face 5 (Orange)
                0.5, 0.0, 0.5, // Color for Triangle 1 of Face 6 (Purple)
                0.5, 0.0, 0.5; // Color for Triangle 2 of Face 6 (Purple)

        init();
    }

    virtual void init() override {
        for (int z = 0; z < m_density_depth; ++z) {
            for (int y = 0; y < m_density_height; ++y) {
                for (int x = 0; x < m_density_width; ++x) {
                    // Simple gradient as an example
                        int index = x + y * m_density_width + z * m_density_width * m_density_height;

                    if (x * x + y * y + z * z < 25) {
                        m_density[index] = 0.1;
                    } else {
                        m_density[index] = 0.0;
                    }
                }
            }
        }

        reset();
    }

    virtual void resetMembers() override {
    }

    virtual void updateRenderGeometry() override {
    }

    virtual bool advance() override {
        // advance m_time
        m_time += m_dt;
        m_step++;

        return false;
    }

    bool rayIntersectsBox(const Eigen::Vector3f rayOrigin, const Eigen::Vector3f rayDir,
        const Eigen::Vector3f boxMin, const Eigen::Vector3f boxMax, Eigen::Vector3f& entryPoint, Eigen::Vector3f& exitPoint) {
        float tMin = (boxMin.x() - rayOrigin.x()) / rayDir.x();
        float tMax = (boxMax.x() - rayOrigin.x()) / rayDir.x();

        if (tMin > tMax) std::swap(tMin, tMax);

        float tyMin = (boxMin.y() - rayOrigin.y()) / rayDir.y();
        float tyMax = (boxMax.y() - rayOrigin.y()) / rayDir.y();

        if (tyMin > tyMax) std::swap(tyMin, tyMax);

        if ((tMin > tyMax) || (tyMin > tMax))
            return false; // No intersection

        if (tyMin > tMin)
            tMin = tyMin;

        if (tyMax < tMax)
            tMax = tyMax;

        float tzMin = (boxMin.z() - rayOrigin.z()) / rayDir.z();
        float tzMax = (boxMax.z() - rayOrigin.z()) / rayDir.z();

        if (tzMin > tzMax) std::swap(tzMin, tzMax);

        if ((tMin > tzMax) || (tzMin > tMax))
            return false; // No intersection

        if (tzMin > tMin)
            tMin = tzMin;

        if (tzMax < tMax)
            tMax = tzMax;

        // Calculate entry and exit points
        entryPoint = rayOrigin + rayDir * tMin;
        exitPoint = rayOrigin + rayDir * tMax;

        return true; // Intersection occurred
    }

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        viewer.data().set_mesh(10 * m_renderV, m_renderF);
        viewer.data().set_colors(m_renderC);

        // Vertex Shader
        const char *vertex_shader_code = R"(
            #version 330 core
            layout(location = 0) in vec3 position;
            layout(location = 1) in vec3 color;

            out vec3 texCoord;
            out vec3 fragWorldPosition;

            uniform mat4 model;
            uniform mat4 view;
            uniform mat4 projection;

            void main() {
                texCoord = position;  // Assume normalized device coordinates for simplicity
                gl_Position = projection * view * model * vec4(position, 1.0);
                fragWorldPosition = position;
            }
        )";

        // Fragment Shader
        const char *fragment_shader_code = R"(
            #version 330 core
            in vec3 texCoord;
            in vec3 fragWorldPosition;

            uniform sampler3D myTexture;

            out vec4 color;

            void main() {
                // Sample the 3D texture using normalized texture coordinates
                float density = texture(myTexture, texCoord).r;  // Get the red channel for grayscale

                // Map density to grayscale color output
                color = vec4(fragWorldPosition / 10, 1.0);
            }
        )";

        // Compile shaders and link program
        GLuint shaderProgram = glCreateProgram();
        GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex_shader, 1, &vertex_shader_code, NULL);
        glCompileShader(vertex_shader);
        glAttachShader(shaderProgram, vertex_shader);

        GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment_shader, 1, &fragment_shader_code, NULL);
        glCompileShader(fragment_shader);
        glAttachShader(shaderProgram, fragment_shader);

        glLinkProgram(shaderProgram);

        // Clean up shaders (they’re linked into the program, no longer needed)
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);

        viewer.data().meshgl.shader_mesh = shaderProgram;

        // Set up transformation matrices
        GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
        GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
        GLint projLoc = glGetUniformLocation(shaderProgram, "projection");

        // Use Eigen to access the current view/projection from the viewer
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f view = viewer.core.view;
        Eigen::Matrix4f projection = viewer.core.proj;

        // Activate shader program and set uniform matrices

        glUseProgram(shaderProgram);
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, model.data());
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, view.data());
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, projection.data());

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
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, m_density_depth, m_density_height, m_density_depth, 0, GL_RED, GL_FLOAT,
                     m_density.data());

        glBindTexture(GL_TEXTURE_3D, 0); // Unbind the texture

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, textureID);
        // Set the sampler3D uniform to use texture unit 0
        glUniform1i(glGetUniformLocation(shaderProgram, "myTexture"), 0);

        // Bind the mesh VAO and render
        glBindVertexArray(viewer.data().meshgl.vao_mesh);
        glDrawElements(GL_TRIANGLES, m_renderF.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        // Clean up shader program
        glUseProgram(0);
        glDeleteProgram(shaderProgram);
    }

private:
    Eigen::MatrixXd m_renderV;
    Eigen::MatrixXi m_renderF;
    Eigen::MatrixXd m_renderC;

    size_t m_density_width;
    size_t m_density_height;
    size_t m_density_depth;
    std::vector<float> m_density;
};
