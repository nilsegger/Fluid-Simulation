#include <complex>
#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>

#include "Renderer.h"

const int RES_X = 32;
const int RES_Y = 32;
const int RES_Z = 32;

const int WIDTH = 768;
const int HEIGHT = 768;

class FluidSim {
public:
    FluidSim(): m_vu(RES_X * RES_Y * RES_Z, 0.0f), m_vv(RES_X * RES_Y * RES_Z, 0.0f),
                m_vw(RES_X * RES_Y * RES_Z, 0.0f), m_s(RES_X * RES_Y * RES_Z, 1.0f),
                m_density(RES_X * RES_Y * RES_Z, 0.0) {
        for (int i = 0; i < RES_X * RES_Y * RES_Z; i++) {
            int x, y, z;
            inverseIndex(i, x, y, z);
            if (y == 0 || y == RES_Y - 1) {
                m_s[i] = 0.0f;
            } else {
                m_s[i] = 1.0f;
            }
        }
    }

    void create_sphere_density() {
        iterate([this](const int i, const int x, const int y, const int z) {
            int xx = x - RES_X / 2;
            int yy = y - RES_Y / 2;
            int zz = z - RES_Z / 2;
            if (xx * xx + yy * yy + zz * zz < 16 * 16) {
                m_density[i] = float(16 * 16 - (xx * xx + yy * yy + zz * zz)) / float(16 * 16) * 0.1;
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

            if(std::pow(x1 - x, 2.0f) + std::pow(y1 - y, 2.0f) + std::pow(z1 - z, 2.0f) <= 16) {
                m_density[i] += 0.1 * t_step;
                m_vv[i] += t_step * 64.0f;
            }
        });
    }

    const std::vector<float>& get_density() {
        return m_density;
    }

    void update(float t_step) {
        // Great video https://www.youtube.com/watch?v=iKAVRgIrUOU
        // 1. Modify velocity values (e.g. gravity)
        iterate([t_step, this](const int i, const int x, const int y, const int z) {
            m_vv[i] = m_vv[i] + (t_step * -9.81f);
        });

        std::vector ss_vv(m_vv);
        std::vector ss_vu(m_vu);
        std::vector ss_vw(m_vw);
        std::vector ss_d(m_density);

        // 3. Move the velocity field (advection)
        iterate([this, t_step, &ss_vv, &ss_vu, &ss_vw, &ss_d](const int i, const int x, const int y, const int z) {
            // semi-lagrangian

            // TODO: check for obstacles

            // Backtrack, find out where "particles" come from
            m_vu[i] = sample(ss_vu, ss_vv, ss_vw, i, x, y, z, SampleDirection::X, t_step);
            m_vv[i] = sample(ss_vu, ss_vv, ss_vw, i, x, y, z, SampleDirection::Y, t_step);
            m_vw[i] = sample(ss_vu, ss_vv, ss_vw, i, x, y, z, SampleDirection::Z, t_step);

            // Same backtracking based on the same velocities
            m_density[i] = std::min(sample_density(ss_d, ss_vu, ss_vv, ss_vw, x, y, z, t_step), 1.0f);
        });

        // 2. make fluid incompressable (projection)
        // TODO: why does this step not matter. Divergence always 0????????????????????????????

        for(int i = 0; i < 100; i++) {
            iterate([this](const int i, const int x, const int y, const int z) {

                if(m_s[i] == 0) {
                    return;
                }

                int ixpp = index(x + 1, y, z);
                int ixmm = index(x - 1, y, z);
                int iypp = index(x, y + 1, z);
                int iymm = index(x, y - 1, z);
                int izpp = index(x, y, z + 1);
                int izmm = index(x, y, z - 1);

                // Gauss-Seidel method
                /*
                float divergence = m_vu[i] - safeSample(m_vu, ixpp, m_vu[i]) +
                                   m_vv[i] - safeSample(m_vv, iypp, m_vv[i]) +
                                   m_vw[i] - safeSample(m_vw, izpp, m_vw[i]);
                */

                float divergence = 0.0f;

                divergence += m_vu[i] - safeSample(m_vu, ixpp, 0.0f) +
                              m_vv[i] - safeSample(m_vv, iypp, 0.0f) +
                              m_vw[i] - safeSample(m_vw, izpp, 0.0f);

                if (divergence == 0) {
                    return;
                }

                float s = m_s[ixpp] + m_s[ixmm] + m_s[iypp] + m_s[iymm] + m_s[izpp] + m_s[izmm];

               // std::cerr << divergence << '\n';

                float p = divergence / s;

                m_vu[i] -= m_s[ixmm] * p;
                m_vu[ixpp] += m_s[ixpp] * p;
                m_vv[i] -= m_s[iymm] * p;
                m_vv[iypp] += m_s[iypp] * p;
                m_vw[i] -= m_s[izmm] * p;
                m_vw[izpp] += m_s[izpp] * p;
            });

        }
    }

    void draw_velocity_field(Line &l, Line &l1, Line &l2) {
        iterate([this, &l, &l1, &l2](const int i, const int x, const int y, const int z) {
            if (z != 1) return;

            float max_w = 0.8f / RES_X;
            float max_h = .8f / RES_Y;

            float uv_x = (float(x) / float(RES_X) - 0.5f) * 2.0f;
            float uv_y = (float(y) / float(RES_Y) - 0.5f) * 2.0f;
            float uv_z = (float(z) / float(RES_Z)) * 2.0f;

            Eigen::Transform<float, 3, Eigen::Affine> transform_x = Eigen::Transform<float, 3,
                Eigen::Affine>::Identity();
            float sx = std::copysign(std::min(std::abs(m_vu[i] * max_w), max_w), m_vu[i]);
            transform_x.translate(Eigen::Vector3f(uv_x, uv_y + (.5f / RES_Y), uv_z));
            transform_x.scale(sx);
            l.draw(transform_x.matrix(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
            l1.draw(transform_x.matrix(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
            l2.draw(transform_x.matrix(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));

            Eigen::Transform<float, 3, Eigen::Affine> transform_y = Eigen::Transform<float, 3,
                Eigen::Affine>::Identity();
            float sy = std::copysign(std::min(std::abs(m_vv[i] * max_h), max_h), m_vv[i]);
            transform_y.translate(Eigen::Vector3f(uv_x + (.5f / RES_X), uv_y, uv_z));
            transform_y.scale(sy);
            float angle = 90.0f * M_PI / 180.0f; // Convert degrees to radians
            transform_y.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
            l.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
            l1.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
            l2.draw(transform_y.matrix(), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
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
        int i = x + y * (RES_X) + z * (RES_X) * (RES_Y);
        if (i < 0 || i >= RES_X * RES_Y * RES_Z) {
            return -1;
        }
        return i;
    }

    static void inverseIndex(int index, int &x, int &y, int &z) {
        z = index / (RES_X * RES_Y);
        int remainder = index % (RES_X * RES_Y);
        y = remainder / RES_X;
        x = remainder % RES_X;
    }

    template<typename F>
    void iterate(const F &func) {
        for (int i = 0; i < RES_X * RES_Y * RES_Z; i++) {
            int x, y, z;
            inverseIndex(i, x, y, z);
            if (x != 0 && x < RES_X - 1 && y != 0 && y < RES_Y - 1 && z != 0 && z < RES_Z - 1)
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
};


int main() {
    std::ios_base::sync_with_stdio(false);

    Renderer renderer;
    if(renderer.open(WIDTH, HEIGHT) == -1) {
        return -1;
    };

    FluidSim sim;
    // sim.create_sphere_density();

    Line l(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    Line l1(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(.6f, 0.2f, 0.0f));
    Line l2(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(.6f, -0.2f, 0.0f));

    double deltaTime = 0.0;

    // Render loop
    while (!renderer.windowHasBeenClosed()) {
        auto startTime = std::chrono::high_resolution_clock::now();

        sim.sphere_influence(deltaTime / 1000.0f);
        sim.update(std::min(deltaTime / 1000.0f, 1.0 / 30.0));

        renderer.clear_frame();

        renderer.bind_fluid_shader_data(WIDTH, HEIGHT, RES_X, RES_Y, RES_Z, sim.get_density());

        renderer.draw_fluid_quad();

        // sim.draw_velocity_field(l, l1, l2);

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = endTime - startTime;
        deltaTime = elapsed.count();

        // Print or log the frame time in milliseconds
        if (deltaTime / 1000.f > 1.0 / 30.0) std::cerr << "Simulation lagging behind.\n";
        std::cout << "Frame time: " << deltaTime << " ms." << std::endl;
    }

    // Clean up
    renderer.close();
    return 0;
}
