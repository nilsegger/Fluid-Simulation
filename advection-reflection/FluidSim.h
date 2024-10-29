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
    FluidSim() : Simulation(), m_renderV(8, 3), m_renderF(12, 3), m_renderC(12, 3) { init(); }

    virtual void init() override {

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

    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {

        m_renderV <<
            -0.5, -0.5, -0.5,  // Vertex 0
             0.5, -0.5, -0.5,  // Vertex 1
             0.5,  0.5, -0.5,  // Vertex 2
            -0.5,  0.5, -0.5,  // Vertex 3
            -0.5, -0.5,  0.5,  // Vertex 4
             0.5, -0.5,  0.5,  // Vertex 5
             0.5,  0.5,  0.5,  // Vertex 6
            -0.5,  0.5,  0.5;  // Vertex 7

        // Define face indices for the 12 triangles (6 faces, 2 triangles per face)
        m_renderF <<
            0, 1, 2,  // Triangle 1 of Face 1 (front)
            0, 2, 3,  // Triangle 2 of Face 1 (front)
            4, 5, 6,  // Triangle 1 of Face 2 (back)
            4, 6, 7,  // Triangle 2 of Face 2 (back)
            0, 1, 5,  // Triangle 1 of Face 3 (bottom)
            0, 5, 4,  // Triangle 2 of Face 3 (bottom)
            2, 3, 7,  // Triangle 1 of Face 4 (top)
            2, 7, 6,  // Triangle 2 of Face 4 (top)
            0, 3, 7,  // Triangle 1 of Face 5 (left)
            0, 7, 4,  // Triangle 2 of Face 5 (left)
            1, 2, 6,  // Triangle 1 of Face 6 (right)
            1, 6, 5;  // Triangle 2 of Face 6 (right)

        // Define colors for each triangle (optional)

        m_renderC <<
            1.0, 0.0, 0.0,  // Color for Triangle 1 of Face 1 (Red)
            1.0, 0.0, 0.0,  // Color for Triangle 2 of Face 1 (Red)
            0.0, 1.0, 0.0,  // Color for Triangle 1 of Face 2 (Green)
            0.0, 1.0, 0.0,  // Color for Triangle 2 of Face 2 (Green)
            0.0, 0.0, 1.0,  // Color for Triangle 1 of Face 3 (Blue)
            0.0, 0.0, 1.0,  // Color for Triangle 2 of Face 3 (Blue)
            1.0, 1.0, 0.0,  // Color for Triangle 1 of Face 4 (Yellow)
            1.0, 1.0, 0.0,  // Color for Triangle 2 of Face 4 (Yellow)
            1.0, 0.5, 0.0,  // Color for Triangle 1 of Face 5 (Orange)
            1.0, 0.5, 0.0,  // Color for Triangle 2 of Face 5 (Orange)
            0.5, 0.0, 0.5,  // Color for Triangle 1 of Face 6 (Purple)
            0.5, 0.0, 0.5;  // Color for Triangle 2 of Face 6 (Purple)

        viewer.data().set_mesh(m_renderV, m_renderF);
        viewer.data().set_colors(m_renderC);
    }

private:
    Eigen::MatrixXd m_renderV;
    Eigen::MatrixXi m_renderF;
    Eigen::MatrixXd m_renderC;
};
