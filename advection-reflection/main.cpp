#include <complex>
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>

#include "Renderer.h"

const int RES_X = 16;
const int RES_Y = 16;
const int RES_Z = 16;

const int WIDTH = 768;
const int HEIGHT = 768;

const float DX = 1.0f;
const float DY = 1.0f;
const float DZ = 1.0f;

class FluidSim {
public:
    FluidSim(): m_vu(RES_X * RES_Y * RES_Z, 0.0f), m_vv(RES_X * RES_Y * RES_Z, 0.0f),
                m_vw(RES_X * RES_Y * RES_Z, 0.0f), m_s(RES_X * RES_Y * RES_Z, CellType::Fluid),
                m_density(RES_X * RES_Y * RES_Z, 0.0), m_pressure(RES_X * RES_Y * RES_Z, 0.0f) {
        for (int i = 0; i < RES_X * RES_Y * RES_Z; i++) {
            if (isBoundary(i)) {
                m_s[i] = CellType::Object;
            } else {
                m_s[i] = CellType::Fluid;
            }

            int x,y,z;
            inverseIndex(i ,x, y, z);

            if(index(x,y,z) != i) {
                std::cerr << "index error" << std::endl;
            }

        }
    }

    void create_sphere_density() {
        iterate([this](const int i, const int x, const int y, const int z) {
            int xx = x - RES_X / 2;
            int yy = y - RES_Y / 2;
            int zz = z - RES_Z / 2;
            if (xx * xx + yy * yy + zz * zz < 8 * 8) {
                m_density[i] = float(8 * 8 - (xx * xx + yy * yy + zz * zz)) / float(8 * 8) * 0.1;
            } else {
                m_density[i] = 0.0;
            }
        });
    }

    void sphere_influence(float t_step) {
        iterate([this, t_step](const int i, const int x, const int y, const int z) {
            float x1 = RES_X / 2.0f;
            float y1 = RES_Y / 8.0f;
            float z1 = RES_Z / 2.0f;

            if (std::pow(x1 - x, 2.0f) + std::pow(y1 - y, 2.0f) + std::pow(z1 - z, 2.0f) <= 16) {
                m_density[i] += 0.1 * t_step;
                m_vv[i] += t_step * 64.0f;
            }
        });
    }

    void bottom_influence() {
        iterate([this](const int i, const int x, const int y, const int z) {
            if (y == 1) {
                m_vv[i] = 0.5f;
                m_s[i] = CellType::StaticFluid;
            }

            if(y == RES_Y - 2) {
                m_vv[i] = -0.5f;
                m_s[i] = CellType::StaticFluid;
            }
        });
    }

    void clear() {
        iterate([this](const int i, const int x, const int y, const int z) {
           m_vu[i] = 0.0f;
           m_vv[i] = 0.0f;
           m_vw[i] = 0.0f;
            m_density[i] = 0.0f;
        });
    }

    const std::vector<float> &get_density() {
        return m_density;
    }

    void update(float t_step) {
        // Great video https://www.youtube.com/watch?v=iKAVRgIrUOU
        // 1. Modify velocity values (e.g. gravity)


        iterate([t_step, this](const int i, const int x, const int y, const int z) {

            if(m_s[i] == CellType::Fluid) {
                //m_vv[i] -= t_step * 9.81; // m_vv[i] //+ (t_step * -9.81f);
            }

            // if(y == 1) m_vv[i] = 0.5f;
            // if(x == 1) m_vu[i] = 0.5f;
        });


        // 2. make fluid incompressable (projection)
        for (int i = 0; i < 10; i++) {
             gauss_seidel_projection();
        }

        std::cerr << "Average Divergence: " << avgDivergence() / (RES_X * RES_Y * RES_Z) << '\n';

        semi_lagrange(t_step);
    }

    void draw_velocity_field(Line &l, Line &l1, Line &l2, Line &l3, Line &l4, const int depth) {
        float max_norm = 0.00001f;

        iterate([this, &depth, &max_norm](const int i, const int x, const int y, const int z) {
            if (z != depth) return;
            float u = (m_vu[i] + m_vu[index(x + 1, y, z)]) / 2.0f;
            float v = (m_vv[i] + m_vv[index(x, y + 1, z)]) / 2.0f;
            float w = (m_vw[i] + m_vw[index(x, y, z + 1)]) / 2.0f;
            Eigen::Vector3f direction(u, v, w);
            float norm = direction.norm();
            max_norm = std::max(norm, max_norm);
        });

        iterate([this, &l, &l1, &l2, &l3, &l4, &depth, &max_norm](const int i, const int x, const int y, const int z) {
            if (z != depth) return;

            float max_w = 0.8f / RES_X;

            float u = (m_vu[i] + m_vu[index(x + 1, y, z)]) / 2.0f;
            float v = (m_vv[i] + m_vv[index(x, y + 1, z)]) / 2.0f;
            float w = (m_vw[i] + m_vw[index(x, y, z + 1)]) / 2.0f;

            float uv_x = (float(x) / float(RES_X) - 0.5f) * 2.0f + (0.5f * max_w);
            float uv_y = (float(y) / float(RES_Y) - 0.5f) * 2.0f + (0.5f * max_w);
            float uv_z = 0.0f; // (float(z) / float(RES_Z)) * 2.0f;

            Eigen::Vector3f x_axis(1.0f, 0, 0.0f);
            Eigen::Vector3f direction(u, v, w);
            float norm = direction.norm();

            if (norm == 0.0f) return;

            direction.normalize();
            Eigen::Vector3f axis = x_axis.cross(direction);
            double angle = std::acos(x_axis.dot(direction));
            axis.normalize();
            Eigen::AngleAxisf rotation(angle, axis);

            Eigen::Transform<float, 3, Eigen::Affine> transform_y = Eigen::Transform<float, 3,
                Eigen::Affine>::Identity();

            transform_y.translate(Eigen::Vector3f(uv_x, uv_y, uv_z));
            transform_y.rotate(rotation);
            transform_y.scale(max_w / max_norm * norm);

            //float sy = std::copysign(std::min(std::abs(m_vv[i] * max_h), max_h), m_vv[i]);

            l.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 0.0f, 1.0f));
            l1.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
            l2.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
            l3.draw(transform_y.matrix(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
            l4.draw(transform_y.matrix(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        });
    }

    void gauss_seidel_projection() {
        iterate([this](const int i, const int x, const int y, const int z) {
            if (m_s[i] == CellType::Object) return;

            int ixpp = index(x + 1, y, z);
            int ixmm = index(x - 1, y, z);
            int iypp = index(x, y + 1, z);
            int iymm = index(x, y - 1, z);
            int izpp = index(x, y, z + 1);
            int izmm = index(x, y, z - 1);

            // Gauss-Seidel method

            float s = 0.0f;
            if(m_s[ixpp] == CellType::Fluid) s++;
            if(m_s[ixmm] == CellType::Fluid) s++;
            if(m_s[iypp] == CellType::Fluid) s++;
            if(m_s[iymm] == CellType::Fluid) s++;
            if(m_s[izpp] == CellType::Fluid) s++;
            if(m_s[izmm] == CellType::Fluid) s++;

            float divergence = (m_vu[ixpp] - m_vu[i]) / DX +
                               (m_vv[iypp] - m_vv[i]) / DY +
                               (m_vw[izpp] - m_vw[i]) / DZ;
            float dd = divergence / s;

            if (m_s[ixmm] == CellType::Fluid && m_s[i] == CellType::Fluid) m_vu[i] += dd;
            if (m_s[ixpp] == CellType::Fluid) m_vu[ixpp] -= dd;
            if (m_s[iymm] == CellType::Fluid && m_s[i] == CellType::Fluid) m_vv[i] += dd;
            if (m_s[iypp] == CellType::Fluid) m_vv[iypp] -= dd;
            if (m_s[izmm] == CellType::Fluid && m_s[i] == CellType::Fluid) m_vw[i] += dd;
            if (m_s[izpp] == CellType::Fluid) m_vw[izpp] -= dd;
        });
    }

    void semi_lagrange(const float t_step) {
        std::vector ss_vv(m_vv);
        std::vector ss_vu(m_vu);
        std::vector ss_vw(m_vw);
        std::vector ss_d(m_density);

        iterate([this, &ss_vv, &ss_vw, &ss_vu, &ss_d, &t_step](const int i, const int x, const int y, const int z) {

            m_vu[i] = sample(ss_vu, ss_vv, ss_vw, i, x, y, z, SampleDirection::X, t_step);
            m_vv[i] = sample(ss_vu, ss_vv, ss_vw, i, x, y, z, SampleDirection::Y, t_step);
            m_vw[i] = sample(ss_vu, ss_vv, ss_vw, i, x, y, z, SampleDirection::Z, t_step);

            // Same backtracking based on the same velocities
            //m_density[i] = sample_density(ss_d, ss_vu, ss_vv, ss_vw, x, y, z, t_step);

        });
    }

    float avgDivergence() {
        float div = 0.0f;
        float count = 0.0f;
        iterate([this, &div, &count](const int i, const int x, const int y, const int z) {
            if(m_s[i] == CellType::Fluid) { // Only count non static fields
                int ixpp = index(x + 1, y, z);
                int iypp = index(x, y + 1, z);
                int izpp = index(x, y, z + 1);
                float divergence = (m_vu[ixpp] - m_vu[i]) / DX +
                                   (m_vv[iypp] - m_vv[i]) / DY +
                                   (m_vw[izpp] - m_vw[i]) / DZ;
                div += divergence;
                count++;
            }
        });
        return div / count;
    }

private:

    enum CellType {
        Fluid,
        Object,
        StaticFluid // Source / Sink?
    };

    // Should be represeneted as a staggered grid. velocity vectors are not in the center of the cell
    // https://www.researchgate.net/profile/Ke-Qing-Xia/publication/325053508/figure/fig1/AS:625094766370816@1526045614156/Schematic-of-the-staggered-grid_W640.jpg
    std::vector<float> m_vu;
    std::vector<float> m_vv;
    std::vector<float> m_vw;
    std::vector<float> m_pressure;

    // 1 means fluid cell, 0 means obstable
    std::vector<CellType> m_s;

    std::vector<float> m_density;

    float m_advection_speed = 1.0f;



    // staggered grid index
    static int index(int x, int y, int z) {
        return (z * RES_X * RES_Y) + (y * RES_X) + x;
    }

    static void inverseIndex(const int index, int &x, int &y, int &z) {
        z = index / (RES_X * RES_Y);
        const int z_indexed = index - (z*RES_X*RES_Y);
        y = z_indexed / RES_X;
        x = z_indexed % RES_X;
    }

    template<typename F>
    void iterate(const F &func) {
        for (int i = 0; i < RES_X * RES_Y * RES_Z; i++) {
            int x, y, z;
            inverseIndex(i, x, y, z);
            if (x > 0 && x < RES_X - 1 && y > 0 && y < RES_Y - 1 && z > 0 && z < RES_Z - 1)
                func(i, x, y, z);
        }
    }

    enum SampleDirection {
        X, Y, Z
    };

    // Move back by new velocity and uniformly sample value
    float sample(std::vector<float> &ss_vu, std::vector<float> &ss_vv, std::vector<float> &ss_vw, const int i,
                 const int x, const int y, const int z,
                 SampleDirection direction, float t_step) {
        // Vector to move back by, different depending on sample direction
        float vx = safeSample(ss_vu, i, 0.0f) * t_step;
        float vy = safeSample(ss_vv, i, 0.0f) * t_step;
        float vz = safeSample(ss_vw, i, 0.0f) * t_step;

        switch (direction) {
            case X: {
                vy = (safeSample(ss_vv, i, 0.0f) + safeSample(ss_vv, x - 1, y, z, 0.0f) +
                      safeSample(ss_vv, x, y + 1, z, 0.0f) + safeSample(ss_vv, x - 1, y + 1, z, 0.0f))
                     / 4.0f;
                vz = (safeSample(ss_vw, i, 0.0f) + safeSample(ss_vw, x - 1, y, z, 0.0f) +
                      safeSample(ss_vw, x, y, z + 1, 0.0f) + safeSample(ss_vw, x - 1, y, z + 1, 0.0f))
                     / 4.0f;
                break;
            }
            case Y: {
                vx = (safeSample(ss_vu, i, 0.0f) + safeSample(ss_vu, x + 1, y, z, 0.0f) +
                      safeSample(ss_vu, x, y - 1, z, 0.0f) + safeSample(ss_vu, x + 1, y - 1, z, 0.0f))
                     / 4.0f;
                vz = (safeSample(ss_vw, i, 0.0f) + safeSample(ss_vw, x, y, z + 1, 0.0f) +
                      safeSample(ss_vw, x, y - 1, z, 0.0f) + safeSample(ss_vw, x, y - 1, z + 1, 0.0f))
                     / 4.0f;
                break;
            }
            case Z: {
                vx = (safeSample(ss_vu, i, 0.0f) + safeSample(ss_vu, x + 1, y, z, 0.0f) +
                      safeSample(ss_vu, x, y, z - 1, 0.0f) + safeSample(ss_vu, x + 1, y, z - 1, 0.0f))
                     / 4.0f;
                vy = (safeSample(ss_vv, i, 0.0f) + safeSample(ss_vv, x, y, z - 1, 0.0f) +
                      safeSample(ss_vv, x, y + 1, z, 0.0f) + safeSample(ss_vv, x, y + 1, z - 1, 0.0f))
                     / 4.0f;
                break;
            }
        }

        // Move back
        float nu = static_cast<float>(x) - vx;
        float nv = static_cast<float>(x) - vy;
        float nw = static_cast<float>(x) - vz;

        switch (direction) {
            case X: return interpolate(nu, nv, nw, ss_vu);
            case Y: return interpolate(nu, nv, nw, ss_vv);
            case Z: return interpolate(nu, nv, nw, ss_vw);
        }

        return 0.0f;
    }

    // Move back by new velocity and uniformly sample value
    float sample_density(std::vector<float> &density, std::vector<float> &ss_vu, std::vector<float> &ss_vv,
                         std::vector<float> &ss_vw, int x, int y, int z, float t_step) {
        int i = index(x, y, z);

        // Vector to move back by, different depending on sample direction
        float vx = (ss_vu[i] + ss_vu[index(x + 1, y, z)]) / 2.0f * t_step;
        float vy = (ss_vv[i] + ss_vv[index(x, y + 1, z)]) / 2.0f * t_step;
        float vz = (ss_vw[i] + ss_vw[index(x, y, z + 1)]) / 2.0f * t_step;


        // Move back, lands within 4 cells origins -> interpolate
        float nx = static_cast<float>(x) + vx;
        float ny = static_cast<float>(y) + vy;
        float nz = static_cast<float>(z) + vz;

        return interpolate(nx, ny, nz, density);
    }


    float interpolate(const float nx, const float ny, const float nz, const std::vector<float> &samples) {
        float x1 = std::floor(nx);
        float x2 = x1 + 1;
        float y1 = std::floor(ny);
        float y2 = y1 + 1;
        float z1 = std::floor(nz);
        float z2 = z1 + 1;

        float u = (nx - x1) / (x2 - x1);
        float v = (ny - y1) / (y2 - y1);
        float w = (nz - z1) / (z2 - z1);

        int i1 = index(x1, y1, z1);
        float w1 = (1.0f - u) * (1.0f - v) * (1.0f - w);
        int i2 = index(x2, y1, z1);
        float w2 = u * (1.0f - v) * (1.0f - w);
        int i3 = index(x1, y2, z1);
        float w3 = (1.0f - u) * v * (1.0f - w);
        int i4 = index(x1, y1, z2);
        float w4 = (1.0f - u) * (1.0f - v) * w;
        int i5 = index(x2, y2, z1);
        float w5 = u * v * (1.0f - w);
        int i6 = index(x2, y1, z2);
        float w6 = u * (1.0f - v) * w;
        int i7 = index(x1, y2, z2);
        float w7 = (1.0f - u) * v * w;
        int i8 = index(x2, y2, z2);
        float w8 = u * v * w;

        float result = w1 * safeSample(samples, i1, 0.0f) +
                       w2 * safeSample(samples, i2, 0.0f) +
                       w3 * safeSample(samples, i3, 0.0f) +
                       w4 * safeSample(samples, i4, 0.0f) +
                       w5 * safeSample(samples, i5, 0.0f) +
                       w6 * safeSample(samples, i6, 0.0f) +
                       w7 * safeSample(samples, i7, 0.0f) +
                       w8 * safeSample(samples, i8, 0.0f);

        return result;
    }

    float safeSample(const std::vector<float> &samples, const int x, const int y, const int z, const float d) {
        return safeSample(samples, index(x, y, z), d);
    }

    float safeSample(const std::vector<float> &samples, const int i, const float d) {
        if (i < 0 || i >= RES_X * RES_Y * RES_Z) {
            return d;
        }

        int x, y, z;
        inverseIndex(i, x, y, z);
        if (x == 0 || y == 0 || z == 0 || x == RES_X - 1 || y == RES_Y - 1 || z == RES_Z - 1) {
            return d;
        }

        const float v = samples[i];
        if (std::isnan(v) || std::isinf(v)) {
            return d;
        }
        return v;
    }

    bool isBoundary(const int i) {
        int x, y, z;
        inverseIndex(i, x, y, z);
        return x == 0 || y == 0 || z == 0 || x == RES_X - 1 || y == RES_Y - 1 || z == RES_Z - 1;
    }
};


int main() {
    std::ios_base::sync_with_stdio(false);

    Renderer renderer;
    if (renderer.open(WIDTH, HEIGHT) == -1) {
        return -1;
    };

    std::cout << "Controls\nSpace: Switch between density / fluid.\nS: Single step simulation.\nP: Play simulation.\n1: Projection\n2: Semi-Lagrange" <<
            std::endl;

    FluidSim sim;
    sim.create_sphere_density();
    sim.bottom_influence();

    Line l(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    Line l1(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(.6f, 0.2f, 0.0f));
    Line l2(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(.6f, -0.2f, 0.0f));
    Line l3(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(.6f, 0.0f, -0.2f));
    Line l4(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(.6f, 0.0f, 0.2f));

    double deltaTime = 0.0;

    bool renderFluid = false;
    int fieldRenderDepth = 1;
    bool step_only = true;

    // Render loop
    while (!renderer.windowHasBeenClosed()) {
        if (renderer.checkSpaceKeyReleased()) {
            renderFluid = !renderFluid;
        }

        if (renderer.pPressed()) step_only = !step_only;

        if (renderer.upPressed()) fieldRenderDepth++;
        if (renderer.downPressed()) fieldRenderDepth--;

        if(renderer.rPressed()) {
            sim.clear();
            sim.bottom_influence();
        }


        auto startTime = std::chrono::high_resolution_clock::now();

        //sim.sphere_influence(deltaTime / 1000.0f);

        if (!step_only || renderer.sPressed()) {
            sim.update(1.0 / 30.0);

            std::cout << "Rendering layer " << fieldRenderDepth << '\n';
            // Print or log the frame time in milliseconds
            if (deltaTime / 1000.f > 1.0 / 30.0) std::cerr << "Simulation lagging behind.\n";
            std::cout << "Frame time: " << deltaTime << " ms." << std::endl;
        }

        if(renderer.Pressed1()) {
            sim.gauss_seidel_projection();
        }

        if(renderer.Pressed2()) {
            sim.semi_lagrange(1.0 / 30.0);
        }

        renderer.clear_frame();

        if (renderFluid)
            renderer.bind_fluid_shader_data(WIDTH, HEIGHT, RES_X, RES_Y, RES_Z, sim.get_density());
        else {
            sim.draw_velocity_field(l, l1, l2, l3, l4, fieldRenderDepth);
        }

        renderer.draw();

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = endTime - startTime;
        deltaTime = elapsed.count();
    }

    // Clean up
    renderer.close();
    return 0;
}
