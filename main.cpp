#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    constexpr unsigned WINDOW_WIDTH = 1200;
    constexpr unsigned WINDOW_HEIGHT = 780;
    constexpr double PI = 3.14159265358979323846;
    constexpr double DT = 0.002;
    constexpr double TRAIL_INTERVAL = 0.018;
    constexpr std::size_t MAX_TRAIL_POINTS = 1800;

    double toRadians(double degrees) { return degrees * PI / 180.0; }
    double toDegrees(double radians) { return radians * 180.0 / PI; }

    struct State
    {
        double theta1{};
        double omega1{};
        double theta2{};
        double omega2{};
    };

    struct Derivative
    {
        double theta1{};
        double omega1{};
        double theta2{};
        double omega2{};
    };

    struct Parameters
    {
        double mass1 = 1.0;
        double mass2 = 1.0;
        double length1 = 1.35;
        double length2 = 1.25;
        double gravity = 9.81;
        double initialAngle1 = 120.0;
        double initialAngle2 = 12.0;
        double perturbation = 0.5;
    };

    class DoublePendulum
    {
    public:
        explicit DoublePendulum(const Parameters& parameters, double secondAngleOffset = 0.0)
            : parameters_(parameters)
        {
            reset(secondAngleOffset);
        }

        void reset(double secondAngleOffset = 0.0)
        {
            state_ = {toRadians(parameters_.initialAngle1), 0.0,
                      toRadians(parameters_.initialAngle2 + secondAngleOffset), 0.0};
            elapsed_ = 0.0;
            accumulator_ = 0.0;
            trail_.clear();
            trailClock_ = 0.0;
        }

        void setParameters(const Parameters& parameters, double secondAngleOffset = 0.0)
        {
            parameters_ = parameters;
            reset(secondAngleOffset);
        }

        void advance(double frameSeconds)
        {
            accumulator_ += std::min(frameSeconds, 0.05);
            while (accumulator_ >= DT)
            {
                integrateRK4(DT);
                accumulator_ -= DT;
                elapsed_ += DT;
                trailClock_ += DT;
                if (trailClock_ >= TRAIL_INTERVAL)
                {
                    trail_.push_back(secondBobPosition());
                    if (trail_.size() > MAX_TRAIL_POINTS)
                        trail_.erase(trail_.begin(), trail_.begin() + 1);
                    trailClock_ = 0.0;
                }
            }
        }

        sf::Vector2f firstBobPosition() const
        {
            const float scale = 105.0f;
            return {static_cast<float>(parameters_.length1 * scale * std::sin(state_.theta1)),
                    static_cast<float>(parameters_.length1 * scale * std::cos(state_.theta1))};
        }

        sf::Vector2f secondBobPosition() const
        {
            const auto p1 = firstBobPosition();
            const float scale = 105.0f;
            return {p1.x + static_cast<float>(parameters_.length2 * scale * std::sin(state_.theta2)),
                    p1.y + static_cast<float>(parameters_.length2 * scale * std::cos(state_.theta2))};
        }

        const State& state() const { return state_; }
        const std::vector<sf::Vector2f>& trail() const { return trail_; }
        double elapsed() const { return elapsed_; }

    private:
        Parameters parameters_;
        State state_{};
        double elapsed_ = 0.0;
        double accumulator_ = 0.0;
        double trailClock_ = 0.0;
        std::vector<sf::Vector2f> trail_;

        Derivative derivative(const State& s) const
        {
            const double m1 = parameters_.mass1;
            const double m2 = parameters_.mass2;
            const double l1 = parameters_.length1;
            const double l2 = parameters_.length2;
            const double g = parameters_.gravity;
            const double delta = s.theta1 - s.theta2;
            const double denominator = 2.0 * m1 + m2 - m2 * std::cos(2.0 * delta);
            const double safeDenominator = std::abs(denominator) < 1e-9 ? 1e-9 : denominator;

            const double alpha1 = (-g * (2.0 * m1 + m2) * std::sin(s.theta1)
                - m2 * g * std::sin(s.theta1 - 2.0 * s.theta2)
                - 2.0 * std::sin(delta) * m2
                    * (s.omega2 * s.omega2 * l2
                       + s.omega1 * s.omega1 * l1 * std::cos(delta)))
                / (l1 * safeDenominator);

            const double alpha2 = (2.0 * std::sin(delta)
                * (s.omega1 * s.omega1 * l1 * (m1 + m2)
                   + g * (m1 + m2) * std::cos(s.theta1)
                   + s.omega2 * s.omega2 * l2 * m2 * std::cos(delta)))
                / (l2 * safeDenominator);

            return {s.omega1, alpha1, s.omega2, alpha2};
        }

        static State addScaled(const State& s, const Derivative& d, double scale)
        {
            return {s.theta1 + d.theta1 * scale,
                    s.omega1 + d.omega1 * scale,
                    s.theta2 + d.theta2 * scale,
                    s.omega2 + d.omega2 * scale};
        }

        void integrateRK4(double dt)
        {
            const Derivative k1 = derivative(state_);
            const Derivative k2 = derivative(addScaled(state_, k1, dt * 0.5));
            const Derivative k3 = derivative(addScaled(state_, k2, dt * 0.5));
            const Derivative k4 = derivative(addScaled(state_, k3, dt));

            state_.theta1 += dt / 6.0 * (k1.theta1 + 2.0 * k2.theta1 + 2.0 * k3.theta1 + k4.theta1);
            state_.omega1 += dt / 6.0 * (k1.omega1 + 2.0 * k2.omega1 + 2.0 * k3.omega1 + k4.omega1);
            state_.theta2 += dt / 6.0 * (k1.theta2 + 2.0 * k2.theta2 + 2.0 * k3.theta2 + k4.theta2);
            state_.omega2 += dt / 6.0 * (k1.omega2 + 2.0 * k2.omega2 + 2.0 * k3.omega2 + k4.omega2);
        }
    };

    bool loadFont(sf::Font& font)
    {
        const std::array<std::string, 4> paths = {
            "C:/Windows/Fonts/arial.ttf",
            "C:/Windows/Fonts/segoeui.ttf",
            "C:/Windows/Fonts/calibri.ttf",
            "arial.ttf"
        };
        for (const auto& path : paths)
            if (font.openFromFile(path)) return true;
        return false;
    }

    void drawText(sf::RenderWindow& window, const sf::Font& font, const std::string& value,
                  unsigned size, sf::Vector2f position, sf::Color color = sf::Color(225, 232, 242))
    {
        sf::Text text(font, value, size);
        text.setPosition(position);
        text.setFillColor(color);
        window.draw(text);
    }

    void drawLine(sf::RenderWindow& window, sf::Vector2f a, sf::Vector2f b,
                  sf::Color color, float thickness = 3.0f)
    {
        const sf::Vector2f diff = b - a;
        const float length = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        sf::RectangleShape line({length, thickness});
        line.setPosition(a);
        line.setRotation(sf::radians(std::atan2(diff.y, diff.x)));
        line.setFillColor(color);
        window.draw(line);
    }

    void drawBob(sf::RenderWindow& window, sf::Vector2f position, float radius, sf::Color color)
    {
        sf::CircleShape bob(radius);
        bob.setOrigin({radius, radius});
        bob.setPosition(position);
        bob.setFillColor(color);
        bob.setOutlineThickness(2.0f);
        bob.setOutlineColor(sf::Color(235, 240, 250));
        window.draw(bob);
    }

    void drawPendulum(sf::RenderWindow& window, const DoublePendulum& pendulum,
                      sf::Vector2f pivot, sf::Color color, bool showTrail)
    {
        if (showTrail)
        {
            for (std::size_t i = 1; i < pendulum.trail().size(); ++i)
            {
                sf::Vertex segment[] = {
    {pivot + pendulum.trail()[i - 1],
     sf::Color{color.r, color.g, color.b, 35}},
    {pivot + pendulum.trail()[i],
     sf::Color{color.r, color.g, color.b, 190}}
                };
                window.draw(segment, 2, sf::PrimitiveType::Lines);
            }
        }

        const sf::Vector2f p1 = pivot + pendulum.firstBobPosition();
        const sf::Vector2f p2 = pivot + pendulum.secondBobPosition();
        drawLine(window, pivot, p1, sf::Color(205, 215, 230), 4.0f);
        drawLine(window, p1, p2, sf::Color(205, 215, 230), 4.0f);
        drawBob(window, pivot, 5.0f, sf::Color(245, 245, 250));
        drawBob(window, p1, 13.0f, color);
        drawBob(window, p2, 16.0f, color);
    }

    std::string number(double value, int precision = 2)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }
}

int main()
{
    sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}), "Double Pendulum");
    window.setFramerateLimit(60);
    window.setVerticalSyncEnabled(true);

    sf::Font font;
    const bool fontLoaded = loadFont(font);

    Parameters parameters;
    DoublePendulum pendulumA(parameters, 0.0);
    DoublePendulum pendulumB(parameters, parameters.perturbation);
    bool paused = false;
    bool trailsVisible = true;
    bool compareMode = true;
    double simulationSpeed = 1.0;
    sf::Clock frameClock;

    const sf::Color background(15, 20, 32);
    const sf::Color panel(25, 33, 49);
    const sf::Color cyan(75, 210, 220);
    const sf::Color orange(255, 166, 77);

    auto resetBoth = [&]() {
        pendulumA.setParameters(parameters, 0.0);
        pendulumB.setParameters(parameters, parameters.perturbation);
    };

    while (window.isOpen())
    {
        const double frameSeconds = frameClock.restart().asSeconds();
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>()) window.close();
            if (const auto* key = event->getIf<sf::Event::KeyPressed>())
            {
                switch (key->code)
                {
                case sf::Keyboard::Key::Escape: window.close(); break;
                case sf::Keyboard::Key::Space: paused = !paused; break;
                case sf::Keyboard::Key::R: resetBoth(); paused = false; break;
                case sf::Keyboard::Key::T: trailsVisible = !trailsVisible; break;
                case sf::Keyboard::Key::C:
                    pendulumA.setParameters(parameters, 0.0);
                    pendulumB.setParameters(parameters, parameters.perturbation);
                    break;
                case sf::Keyboard::Key::Q: parameters.initialAngle1 = std::clamp(parameters.initialAngle1 + 2.0, 20.0, 160.0); resetBoth(); break;
                case sf::Keyboard::Key::A: parameters.initialAngle1 = std::clamp(parameters.initialAngle1 - 2.0, 20.0, 160.0); resetBoth(); break;
                case sf::Keyboard::Key::W: parameters.initialAngle2 = std::clamp(parameters.initialAngle2 + 2.0, -80.0, 80.0); resetBoth(); break;
                case sf::Keyboard::Key::S: parameters.initialAngle2 = std::clamp(parameters.initialAngle2 - 2.0, -80.0, 80.0); resetBoth(); break;
                case sf::Keyboard::Key::Up: parameters.gravity = std::clamp(parameters.gravity + 0.5, 1.0, 20.0); resetBoth(); break;
                case sf::Keyboard::Key::Down: parameters.gravity = std::clamp(parameters.gravity - 0.5, 1.0, 20.0); resetBoth(); break;
                case sf::Keyboard::Key::Right: parameters.perturbation = std::clamp(parameters.perturbation + 0.1, 0.0, 5.0); resetBoth(); break;
                case sf::Keyboard::Key::Left: parameters.perturbation = std::clamp(parameters.perturbation - 0.1, 0.0, 5.0); resetBoth(); break;
                case sf::Keyboard::Key::Num1: simulationSpeed = 0.5; break;
                case sf::Keyboard::Key::Num2: simulationSpeed = 1.0; break;
                case sf::Keyboard::Key::Num3: simulationSpeed = 2.0; break;
                default: break;
                }
            }
        }

        if (!paused)
        {
            const double simDelta = frameSeconds * simulationSpeed;
            pendulumA.advance(simDelta);
            pendulumB.advance(simDelta);
        }

        window.clear(background);

        sf::RectangleShape topPanel({static_cast<float>(WINDOW_WIDTH), 88.0f});
        topPanel.setFillColor(panel);
        window.draw(topPanel);
        drawText(window, font, "DOUBLE PENDULUM", 25, {24.f, 12.f}, cyan);
        drawText(window, font, "Sensitivity to initial conditions - Run A vs Run B", 16, {26.f, 49.f});

        sf::RectangleShape leftPanel({560.0f, 510.0f});
        leftPanel.setPosition({20.f, 105.f});
        leftPanel.setFillColor(sf::Color(20, 27, 41));
        leftPanel.setOutlineThickness(1.0f);
        leftPanel.setOutlineColor(sf::Color(52, 66, 88));
        window.draw(leftPanel);
        sf::RectangleShape rightPanel = leftPanel;
        rightPanel.setPosition({620.f, 105.f});
        window.draw(rightPanel);

        drawText(window, font, "RUN A - reference", 18, {38.f, 116.f}, cyan);
        drawText(window, font, "RUN B - perturbed", 18, {638.f, 116.f}, orange);
        const sf::Vector2f pivotA{300.f, 190.f};
        const sf::Vector2f pivotB{900.f, 190.f};
        drawPendulum(window, pendulumA, pivotA, cyan, trailsVisible);
        drawPendulum(window, pendulumB, pivotB, orange, trailsVisible);

        sf::RectangleShape infoPanel({1160.f, 145.f});
        infoPanel.setPosition({20.f, 630.f});
        infoPanel.setFillColor(panel);
        window.draw(infoPanel);
        drawText(window, font, paused ? "STATUS: PAUSED" : "STATUS: RUNNING", 16, {35.f, 642.f}, paused ? orange : cyan);
        drawText(window, font, "t = " + number(pendulumA.elapsed(), 2) + " s    dt = 0.002 s    method: RK4", 15, {35.f, 670.f});
        drawText(window, font, "Initial angles: theta1 = " + number(parameters.initialAngle1, 1) + " deg, theta2 = " + number(parameters.initialAngle2, 1) + " deg", 15, {35.f, 696.f});
        drawText(window, font, "g = " + number(parameters.gravity, 2) + " m/s^2    Run B perturbation = +" + number(parameters.perturbation, 1) + " deg    speed: " + number(simulationSpeed, 1) + "x", 15, {35.f, 722.f});
        drawText(window, font, "SPACE pause/resume | R restart | T trails | Q/A angle 1 | W/S angle 2 | UP/DOWN gravity", 13, {35.f, 750.f}, sf::Color(185, 198, 216));
        drawText(window, font, "LEFT/RIGHT perturbation | 1/2/3 speed | ESC quit", 13, {720.f, 750.f}, sf::Color(185, 198, 216));

        window.display();
    }
    return 0;
}
