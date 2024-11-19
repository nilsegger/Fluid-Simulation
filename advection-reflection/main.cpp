#include <complex>
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <tbb/tbb.h>

#include "Renderer.h"

#define USE_TBB true

const int RES_X = 32;
const int RES_Y = 32;
const int RES_Z = 32;

const int WIDTH = 768;
const int HEIGHT = 768;

class FluidSim {
public:
    FluidSim(): m_vu(RES_X * RES_Y * RES_Z, 0.0f), m_vv(RES_X * RES_Y * RES_Z, 0.0f),
                m_vw(RES_X * RES_Y * RES_Z, 0.0f), m_ss_vu(RES_X * RES_Y * RES_Z, 0.0f),
                m_ss_vv(RES_X * RES_Y * RES_Z, 0.0f),
                m_ss_vw(RES_X * RES_Y * RES_Z, 0.0f), m_s(RES_X * RES_Y * RES_Z, CellType::Fluid),
                m_density(RES_X * RES_Y * RES_Z, 0.0), m_ss_density(RES_X * RES_Y * RES_Z, 0.0) {
        for (int i = 0; i < RES_X * RES_Y * RES_Z; i++) {
            if (isBoundary(i)) {
                m_s[i] = CellType::Object;
            } else {
                m_s[i] = CellType::Fluid;
            }

            int x, y, z;
            inverseIndex(i, x, y, z);

            if (index(x, y, z) != i) {
                std::cerr << "index error" << std::endl;
            }
        }
    }

    void sphere_influence(const float radius) {
        iterate([this, &radius](const int i, const int x, const int y, const int z) {
            const float x_centered = x - (RES_X / 2.0f);
            const float y_centered = y - radius;
            const float z_centered = z - (RES_Z / 2.0f);

            const float distance = std::pow(x_centered, 2.0f) + std::pow(y_centered, 2.0f) + std::pow(z_centered, 2.0f);
            if (distance < std::pow(radius, 2.0f)) {
                const float w = 1.0f - (distance / std::pow(radius, 2.0f));
                m_s[i] = CellType::StaticFluid;
                m_density[i] = w;
                m_vv[i] = 20.0f * w;
            }
        });
    }

    void fill_sphere_influence(const float radius, const int iter) {
        parallel_iterate([this, &radius, &iter](const int i, const int x, const int y, const int z) {
            const float x_centered = x - (RES_X / 2.0f);
            const float y_centered = y - radius;
            const float z_centered = z - (RES_Z / 2.0f);

            const float distance = std::pow(x_centered, 2.0f) + std::pow(y_centered, 2.0f) + std::pow(z_centered, 2.0f);
            if (distance < std::pow(radius, 2.0f)) {
                const float w = 1.0f - (distance / std::pow(radius, 2.0f));
                if (iter % 100 == 0) {
                    m_density[i] = w;
                }
            }
        });
    }

    void bottom_influence() {
        iterate([this](const int i, const int x, const int y, const int z) {
            m_vu[i] = (1.0f - (static_cast<float>(x) / static_cast<float>(RES_X))) * 1.0f;
            m_vv[i] = (1.0f - (static_cast<float>(y) / static_cast<float>(RES_Y))) * 1.0f;
            m_vw[i] = (1.0f - (static_cast<float>(z) / static_cast<float>(RES_Z))) * 1.0f;


            if (x == 1 || y == 1 || z == 1) {
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

        // 2. make fluid incompressable (projection)
        float change = gauss_seidel_projection();
        int iter = 1;
        /* TODO: smaller than 5.0f or so is dependent on resolution */
        while (change > 5.0f && iter < 1000) {
            iter++;
            change = gauss_seidel_projection();
        }

        if (iter > 10 && iter < 1000) {
            std::cerr << "Required " << iter << " to converge to < 1.0f change." << std::endl;
        } else if (iter == 1000) {
            std::cerr << "Failed to converge gauss seidel within 1000 iterations. Remaining change " << change <<
                    std::endl;
        }


        // Take snapshot
        iterate([this](const int i, const int x, const int y, const int z) {
            m_ss_vu[i] = m_vu[i];
            m_ss_vv[i] = m_vv[i];
            m_ss_vw[i] = m_vw[i];
            m_ss_density[i] = m_density[i];
        });

        semi_lagrange(t_step, m_ss_vu, m_ss_vv, m_ss_vw, m_ss_density);
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

            float max_w = 1.0f / RES_X;

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
            if (direction.x() != 1.0f) {
                transform_y.rotate(rotation);
            }
            transform_y.scale(max_w / max_norm * norm);

            //float sy = std::copysign(std::min(std::abs(m_vv[i] * max_h), max_h), m_vv[i]);

            l.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 0.0f, 1.0f));
            l1.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
            l2.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
            l3.draw(transform_y.matrix(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
            l4.draw(transform_y.matrix(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        });
    }

    float gauss_seidel_projection() {

        // Take snapshot
        parallel_iterate([this](const int i, const int x, const int y, const int z) {
            m_ss_vu[i] = m_vu[i];
            m_ss_vv[i] = m_vv[i];
            m_ss_vw[i] = m_vw[i];
        });

        auto gsp = [this](const int i, const int x, const int y, const int z) {
            if (m_s[i] == CellType::Object) return;

            int ixpp = index(x + 1, y, z);
            int ixmm = index(x - 1, y, z);
            int iypp = index(x, y + 1, z);
            int iymm = index(x, y - 1, z);
            int izpp = index(x, y, z + 1);
            int izmm = index(x, y, z - 1);

            // Gauss-Seidel method
            float s = 0.0f;
            if (m_s[ixpp] == CellType::Fluid) s++;
            if (m_s[ixmm] == CellType::Fluid) s++;
            if (m_s[iypp] == CellType::Fluid) s++;
            if (m_s[iymm] == CellType::Fluid) s++;
            if (m_s[izpp] == CellType::Fluid) s++;
            if (m_s[izmm] == CellType::Fluid) s++;

            float divergence = (m_vu[ixpp] - m_vu[i]) +
                               (m_vv[iypp] - m_vv[i]) +
                               (m_vw[izpp] - m_vw[i]);

            if(s == 0.0f) {
                return;
            }

            float dd = divergence / s;

            if (m_s[ixmm] == CellType::Fluid && m_s[i] == CellType::Fluid) m_vu[i] += dd;
            if (m_s[ixpp] == CellType::Fluid) m_vu[ixpp] -= dd;
            if (m_s[iymm] == CellType::Fluid && m_s[i] == CellType::Fluid) m_vv[i] += dd;
            if (m_s[iypp] == CellType::Fluid) m_vv[iypp] -= dd;
            if (m_s[izmm] == CellType::Fluid && m_s[i] == CellType::Fluid) m_vw[i] += dd;
            if (m_s[izpp] == CellType::Fluid) m_vw[izpp] -= dd;
        };

        iterate(gsp);

        /*
        for (int i = 0; i < 8; i++) {
            parallel_iterate_pass(8, i, gsp);
        }
        */

        float change = 0.0f;

        iterate([&change, this](const int i, const int x, const int y, const int z) {
            change += std::abs(m_ss_vu[i] - m_vu[i]);
            change += std::abs(m_ss_vv[i] - m_vv[i]);
            change += std::abs(m_ss_vw[i] - m_vw[i]);
        });

        // std::cout << change << std::endl;

        return change;
    }

    void semi_lagrange(const float t_step, const std::vector<float> &ss_vu, const std::vector<float> &ss_vv,
                       const std::vector<float> &ss_vw, const std::vector<float> &ss_d) {
        parallel_iterate(
            [this, &ss_vv, &ss_vw, &ss_vu, &ss_d, &t_step](const int i, const int x, const int y, const int z) {
                if (m_s[i] == Fluid) {
                    float dx = ss_vu[i];
                    float dy = (ss_vv[i] + ss_vv[index(x - 1, y, z)] + ss_vv[index(x, y + 1, z)] + ss_vv[index(
                                    x - 1, y + 1, z)]) / 4.0f;
                    float dz = (ss_vw[i] + ss_vw[index(x - 1, y, z)] + ss_vw[index(x, y, z + 1)] + ss_vw[index(
                                    x - 1, y, z + 1)]) / 4.0f;
                    m_vu[i] = sample(x, y, z, dx, dy, dz, t_step, ss_vu);


                    dx = (ss_vu[i] + ss_vu[index(x, y - 1, z)] + ss_vu[index(x + 1, y, z)] + ss_vu[index(
                              x + 1, y - 1, z)]) / 4.0f;
                    dy = ss_vv[i];
                    dz = (ss_vw[i] + ss_vw[index(x, y - 1, z)] + ss_vw[index(x, y, z + 1)] + ss_vw[index(
                              x, y - 1, z + 1)]) / 4.0f;
                    m_vv[i] = sample(x, y, z, dx, dy, dz, t_step, ss_vv);

                    dx = (ss_vu[i] + ss_vu[index(x, y, z - 1)] + ss_vu[index(x + 1, y, z)] + ss_vu[index(
                              x + 1, y, z - 1)]) / 4.0f;
                    dy = (ss_vv[i] + ss_vv[index(x, y, z - 1)] + ss_vv[index(x, y + 1, z)] + ss_vv[index(
                              x, y + 1, z - 1)]) / 4.0f;
                    dz = ss_vw[i];
                    m_vw[i] = sample(x, y, z, dx, dy, dz, t_step, ss_vw);
                }

                if (m_s[i] != Object) {
                    float u = (m_vu[i] + m_vu[index(x + 1, y, z)]) / 2.0f;
                    float v = (m_vv[i] + m_vv[index(x, y + 1, z)]) / 2.0f;
                    float w = (m_vw[i] + m_vw[index(x, y, z + 1)]) / 2.0f;
                    m_density[i] = sample(x, y, z, u, v, w, t_step, ss_d);
                }
            });
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

    std::vector<float> m_ss_vu;
    std::vector<float> m_ss_vv;
    std::vector<float> m_ss_vw;

    // 1 means fluid cell, 0 means obstable
    std::vector<CellType> m_s;

    std::vector<float> m_density;
    std::vector<float> m_ss_density;

    float m_advection_speed = 1.0f;

    // staggered grid index
    static int index(const int x, const int y, const int z) {
        if (x < 0 || y < 0 || z < 0 || x >= RES_X || y >= RES_Y || z >= RES_Z) {
            return -1;
        }

        return (z * RES_X * RES_Y) + (y * RES_X) + x;
    }

    static void inverseIndex(const int index, int &x, int &y, int &z) {
        z = index / (RES_X * RES_Y);
        const int z_indexed = index - (z * RES_X * RES_Y);
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

    template<typename F>
    void parallel_iterate_pass(const int groups, const int group, const F &func) {

        /* TODO: this is wrong and doesnt work */
        if (!USE_TBB) {
            return iterate(func);
        }
        tbb::parallel_for(0, RES_X * RES_Y * RES_Z, [this, &func, &groups, &group](int i) {
            if (i % groups != group) {
                return;
            }

            int x, y, z;
            inverseIndex(i, x, y, z);
            if (x > 0 && x < RES_X - 1 && y > 0 && y < RES_Y - 1 && z > 0 && z < RES_Z - 1)
                func(i, x, y, z);
        });
    }

    template<typename F>
    void parallel_iterate(const F &func) {
        if (!USE_TBB) {
            return iterate(func);
        }
        tbb::parallel_for(0, RES_X * RES_Y * RES_Z, [this, &func](int i) {
            int x, y, z;
            inverseIndex(i, x, y, z);
            if (x > 0 && x < RES_X - 1 && y > 0 && y < RES_Y - 1 && z > 0 && z < RES_Z - 1)
                func(i, x, y, z);
        });
    }

    enum SampleDirection {
        X, Y, Z
    };

    float sample(const float x, const float y, const float z, const float dx, const float dy, const float dz,
                 const float t_step, const std::vector<float> &samples) {
        // Where did particle come from?
        const float nx = static_cast<float>(x) - dx * t_step;
        const float ny = static_cast<float>(y) - dy * t_step;
        const float nz = static_cast<float>(z) - dz * t_step;

        const float bx = std::floor(nx);
        const float tx = bx + 1;
        const float u = (nx - bx) / (tx - bx);

        const float by = std::floor(ny);
        const float ty = by + 1;
        const float v = (ny - by) / (ty - by);

        const float bz = std::floor(nz);
        const float tz = bz + 1;
        const float w = (nz - bz) / (tz - bz);

        float result = 0.0f;
        result += safeSample(samples, bx, by, bz, 0.0f) * ((1.0f - u) * (1.0f - v) * (1.0f - w));
        result += safeSample(samples, tx, by, bz, 0.0f) * ((u) * (1.0f - v) * (1.0f - w));
        result += safeSample(samples, bx, ty, bz, 0.0f) * ((1.0f - u) * (v) * (1.0f - w));
        result += safeSample(samples, bx, by, tz, 0.0f) * ((1.0f - u) * (1.0f - v) * (w));
        result += safeSample(samples, tx, ty, bz, 0.0f) * ((u) * (v) * (1.0f - w));
        result += safeSample(samples, tx, by, tz, 0.0f) * ((u) * (1.0f - v) * (w));
        result += safeSample(samples, bx, ty, tz, 0.0f) * ((1.0f - u) * (v) * (w));
        result += safeSample(samples, tx, ty, tz, 0.0f) * ((u) * (v) * (w));
        return result;
    }

    float safeSample(const std::vector<float> &samples, const int x, const int y, const int z, const float d) {
        if (x < 0 || y < 0 || z < 0 || x >= RES_X || y >= RES_Y || z >= RES_Z) {
            return d;
        }

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
        if (std::isfinite(v)) {
            return v;
        }
        return d;
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

    std::cout <<
            "Controls\nSpace: Switch between density / fluid.\nS: Single step simulation.\nP: Play simulation.\n1: Projection\n2: Semi-Lagrange"
            <<
            std::endl;

    FluidSim sim;
    int iter = 0;
    // sim.create_sphere_density();
    sim.sphere_influence(RES_X / 10.0f);

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

        if (renderer.upPressed()) {
            fieldRenderDepth++;
            std::cout << "Rendering layer " << fieldRenderDepth << std::endl;
        };
        if (renderer.downPressed()) {
            fieldRenderDepth--;
            std::cout << "Rendering layer " << fieldRenderDepth << std::endl;
        }

        if (renderer.rPressed()) {
            sim.clear();
            sim.bottom_influence();
        }


        auto startTime = std::chrono::high_resolution_clock::now();

        //sim.sphere_influence(deltaTime / 1000.0f);

        if (!step_only || renderer.sPressed()) {
            sim.fill_sphere_influence(RES_X / 10.0f, iter);
            iter++;
            sim.update(std::min(static_cast<float>(deltaTime) / 1000.0f, 1.0f / 30.0f));

            // Print or log the frame time in milliseconds
            if (deltaTime / 1000.f > 1.0 / 30.0) std::cerr << "Simulation lagging behind.\n";
            std::cout << "Frame time: " << deltaTime << " ms." << std::endl;
        }

        /*
        if (renderer.Pressed1()) {
            float change = sim.gauss_seidel_projection();
            std::cout << "Changed: " << change << std::endl;
        }

        if (renderer.Pressed2()) {
            sim.semi_lagrange(1.0f / 30.0f);
        }
        */

        renderer.clear_frame();

        if (renderFluid)
            renderer.bind_fluid_shader_data(WIDTH, HEIGHT, RES_X, RES_Y, RES_Z, sim.get_density(), fieldRenderDepth);
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
