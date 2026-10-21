
#include <SFML/Graphics.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <cstdint>
#include <vector>

//1  Створеня зміних

// Структура одного тіла
struct Body
{
    // маса тіла
    double mass;

    sf::Vector2<double> position;
    sf::Vector2<double> velocity;
    sf::Vector2<double> acceleration;

    sf::Color color;

    double radius;

	// список координат, у яких тіло перебувало під час симуляції
    std::vector<sf::Vector2f> trajectory;
};

//Умовна гравітаційна стала моделі
double gravitationalConstant = 1800.0;

// Крок інтегрування, секунди модельного часу
double timeStep = 0.002;

// Масштаб швидкостей для початкових умов
double velocityScale = 1.0;

// М'яке згладжування гравітації поблизу тіл
const double softening = 8.0;

// Стан симуляції.
bool paused = false;
bool showTrajectories = true;
double simulationTime = 0.0;
double initialEnergy = 0.0;
double maximumRelativeEnergyError = 0.0;
// Режим початкових умов
int initialCondition = 1;
std::vector<Body> bodies;


void computeAccelerations();
double calculateKineticEnergy();
double calculatePotentialEnergy();
double calculateTotalEnergy();
void resetSimulation();


// 2 Початкові умови

void resetSimulation()
{
    bodies.clear();
    simulationTime = 0.0;
    maximumRelativeEnergyError = 0.0;

    const sf::Color blue(90, 170, 255);
    const sf::Color orange(255, 170, 70);
    const sf::Color purple(190, 120, 255);
    const sf::Color green(100, 230, 160);

    if (initialCondition == 1)
    {
        // 4 тіла з різними масами
        bodies.push_back({
            4.0,
            {460.0, 400.0},
            {0.0, -32.0 * velocityScale},
            {0.0, 0.0},
            blue,
            12.0,
            {}
            });

        bodies.push_back({
            4.0,
            {740.0, 400.0},
            {0.0, 32.0 * velocityScale},
            {0.0, 0.0},
            orange,
            12.0,
            {}
            });

        bodies.push_back({
            2.0,
            {600.0, 245.0},
            {-35.0 * velocityScale, 0.0},
            {0.0, 0.0},
            purple,
            9.0,
            {}
            });

        bodies.push_back({
            1.0,
            {600.0, 510.0},
            {25.0 * velocityScale, 0.0},
            {0.0, 0.0},
            green,
            7.0,
            {}
            });
    }
    else if (initialCondition == 2)
    {
        // тіла стартують ближче одне до одного
        bodies.push_back({
            5.0,
            {520.0, 400.0},
            {0.0, -22.0 * velocityScale},
            {0.0, 0.0},
            blue,
            13.0,
            {}
            });

        bodies.push_back({
            3.0,
            {680.0, 400.0},
            {0.0, 30.0 * velocityScale},
            {0.0, 0.0},
            orange,
            10.0,
            {}
            });

        bodies.push_back({
            2.0,
            {600.0, 315.0},
            {-30.0 * velocityScale, 0.0},
            {0.0, 0.0},
            purple,
            9.0,
            {}
            });
    }
    else
    {
        // підвищені початкові швидкості
        bodies.push_back({
            4.0,
            {460.0, 400.0},
            {0.0, -55.0 * velocityScale},
            {0.0, 0.0},
            blue,
            12.0,
            {}
            });

        bodies.push_back({
            4.0,
            {740.0, 400.0},
            {0.0, 55.0 * velocityScale},
            {0.0, 0.0},
            orange,
            12.0,
            {}
            });

        bodies.push_back({
            2.0,
            {600.0, 245.0},
            {-60.0 * velocityScale, 0.0},
            {0.0, 0.0},
            purple,
            9.0,
            {}
            });
    }

}


// 3  Обчислення гравітаційних прискорень

void computeAccelerations()
{
    for (auto& body : bodies)
    {
		// Скидаємо прискорення перед обчисленням
        body.acceleration = { 0.0, 0.0 };
    }

    // Розглядаємо кожну пару тіл лише один раз
    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
        {
            const auto delta =
                bodies[j].position - bodies[i].position;

            const double distanceSquared =
                delta.x * delta.x
                + delta.y * delta.y
                + softening * softening;

            const double distanceCubed =
                distanceSquared * std::sqrt(distanceSquared);

            const double factor =
                gravitationalConstant / distanceCubed;

            // Прискорення першого тіла від другого
            bodies[i].acceleration +=
                delta * (factor * bodies[j].mass);

            // Прискорення другого тіла від першого
            bodies[j].acceleration -=
                delta * (factor * bodies[i].mass);
        }
    }
}


// 4 Чисельне інтегрування
void updatePhysics(double dt)
{
    // Половина зміни швидкості під дією прискорення на початку кроку
    for (auto& body : bodies)
    {
        body.velocity += body.acceleration * (dt * 0.5);
    }

    // Переміщення тіл із проміжними швидкостями
    for (auto& body : bodies)
    {
        body.position += body.velocity * dt;
    }

    // Знаходимо прискорення в нових координатах
    computeAccelerations();

    // Друга половина зміни швидкості
    for (auto& body : bodies)
    {
        body.velocity += body.acceleration * (dt * 0.5);
    }

    simulationTime += dt;

    // Зберігаємо траєкторії
    for (auto& body : bodies)
    {
        body.trajectory.push_back({
            static_cast<float>(body.position.x),
            static_cast<float>(body.position.y)
            });

        // Не дозволяємо траєкторіям необмежено збільшуватися
        if (body.trajectory.size() > 3000)
        {
            body.trajectory.erase(
                body.trajectory.begin(),
                body.trajectory.begin() + 100
            );
        }
    }

    // Контролюємо відхилення повної енергії
    const double currentEnergy = calculateTotalEnergy();

    if (std::abs(initialEnergy) > 1e-12)
    {
        const double relativeError =
            std::abs(currentEnergy - initialEnergy)
            / std::abs(initialEnergy);

        maximumRelativeEnergyError =
            std::max(maximumRelativeEnergyError, relativeError);
    }
}


// 5 Енергія системи

double calculateKineticEnergy()
{
    double energy = 0.0;

    for (const auto& body : bodies)
    {
        const double speedSquared =
            body.velocity.x * body.velocity.x
            + body.velocity.y * body.velocity.y;

        energy += 0.5 * body.mass * speedSquared;
    }

    return energy;
}

double calculatePotentialEnergy()
{
    double energy = 0.0;

    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
        {
            const auto delta =
                bodies[j].position - bodies[i].position;

            const double distance =
                std::sqrt(
                    delta.x * delta.x
                    + delta.y * delta.y
                    + softening * softening
                );

            energy -=
                gravitationalConstant
                * bodies[i].mass
                * bodies[j].mass
                / distance;
        }
    }

    return energy;
}

double calculateTotalEnergy()
{
    return calculateKineticEnergy()
        + calculatePotentialEnergy();
}


//6  Допоміжні функції інтерфейсу

// перетворення із заданою кількістю знаків після коми (3)
std::string formatNumber(double value, int precision = 3)
{
    std::ostringstream stream;

    stream << std::fixed
        << std::setprecision(precision)
        << value;

    return stream.str();
}

void drawText(
    sf::RenderWindow& window,
    const sf::Font& font,
    const std::string& content,
    float x,
    float y,
    unsigned int size,
    sf::Color color = sf::Color::White)
{
    sf::Text text(font, content, size);

    text.setFillColor(color);
    text.setPosition({ x, y });

    window.draw(text);
}

void drawTrajectories(sf::RenderWindow& window)
{
    if (!showTrajectories)
    {
        return;
    }

    for (const auto& body : bodies)
    {
        if (body.trajectory.size() < 2)
        {
            continue;
        }

        std::vector<sf::Vertex> vertices;

        vertices.reserve(body.trajectory.size());

        for (const auto& point : body.trajectory)
        {
            vertices.push_back(
                sf::Vertex{
                    point,
                    sf::Color{
                        body.color.r,
                        body.color.g,
                        body.color.b,
                        static_cast<std::uint8_t>(170)
                    }
                }
            );
        }

        window.draw(
            vertices.data(),
            vertices.size(),
            sf::PrimitiveType::LineStrip
        );
    }
}

void drawBodies(sf::RenderWindow& window)
{
    for (const auto& body : bodies)
    {
        sf::CircleShape circle(
            static_cast<float>(body.radius)
        );

		// Встановлюємо центр кола у центр тіла
        circle.setOrigin({
            static_cast<float>(body.radius),
            static_cast<float>(body.radius)
            });

		// Встановлюємо позицію кола у позицію тіла
        circle.setPosition({
            static_cast<float>(body.position.x),
            static_cast<float>(body.position.y)
            });

        circle.setFillColor(body.color);

        // обводка
        circle.setOutlineThickness(1.5f);
        circle.setOutlineColor(sf::Color(235, 240, 255));

        window.draw(circle);
    }
}

void drawInterface(
    sf::RenderWindow& window,
    const sf::Font& font)
{
    const double kinetic = calculateKineticEnergy();
    const double potential = calculatePotentialEnergy();
    const double total = kinetic + potential;

    // Ліва інформаційна панель.
    sf::RectangleShape panel({ 330.f, 800.f });

    panel.setPosition({ 0.f, 0.f });
    panel.setFillColor(sf::Color(24, 28, 38));

    window.draw(panel);

    drawText(
        window, font,
        "Gravity simulation",
        20.f, 20.f, 23,
        sf::Color(110, 190, 255)
    );

    drawText(
        window, font,
        "Variant 33",
        20.f, 57.f, 15,
        sf::Color(180, 190, 210)
    );

    drawText(window, font, "SYSTEM", 20.f, 105.f, 17,
        sf::Color(110, 190, 255));

    drawText(
        window, font,
        "Bodies: " + std::to_string(bodies.size()),
        20.f, 137.f, 16
    );

    drawText(
        window, font,
        "Time: " + formatNumber(simulationTime, 2),
        20.f, 164.f, 16
    );

    drawText(
        window, font,
        paused ? "Status: PAUSED" : "Status: RUNNING",
        20.f, 191.f, 16,
        paused
        ? sf::Color(255, 200, 100)
        : sf::Color(110, 230, 160)
    );

    drawText(window, font, "PARAMETERS", 20.f, 239.f, 17,
        sf::Color(110, 190, 255));

    drawText(
        window, font,
        "G: " + formatNumber(gravitationalConstant, 1),
        20.f, 271.f, 16
    );

    drawText(
        window, font,
        "dt: " + formatNumber(timeStep, 4),
        20.f, 298.f, 16
    );

    drawText(
        window, font,
        "Initial preset: " + std::to_string(initialCondition),
        20.f, 325.f, 16
    );

    drawText(window, font, "ENERGY", 20.f, 373.f, 17,
        sf::Color(110, 190, 255));

    drawText(
        window, font,
        "Kinetic: " + formatNumber(kinetic, 2),
        20.f, 405.f, 15
    );

    drawText(
        window, font,
        "Potential: " + formatNumber(potential, 2),
        20.f, 431.f, 15
    );

    drawText(
        window, font,
        "Total: " + formatNumber(total, 2),
        20.f, 457.f, 15
    );

    drawText(
        window, font,
        "Initial E: " + formatNumber(initialEnergy, 2),
        20.f, 483.f, 15
    );

    drawText(
        window, font,
        "Max energy error: "
        + formatNumber(maximumRelativeEnergyError * 100.0, 4)
        + "%",
        20.f, 510.f, 14,
        sf::Color(255, 205, 120)
    );

    drawText(window, font, "CONTROLS", 20.f, 560.f, 17,
        sf::Color(110, 190, 255));

    drawText(window, font, "Space - Pause / Resume", 20.f, 590.f, 14);
    drawText(window, font, "R - Reset", 20.f, 615.f, 14);
    drawText(window, font, "Up/Down - Change G", 20.f, 640.f, 14);
    drawText(window, font, "Left/Right - Change dt", 20.f, 665.f, 14);
    drawText(window, font, "1 / 2 / 3 - Initial presets", 20.f, 690.f, 14);
    drawText(window, font, "T - Toggle trajectories", 20.f, 715.f, 14);

}


//6  Головна функція
int main()
{
    // Створення вікна
    sf::RenderWindow window(
        sf::VideoMode({ 1200, 800 }),
        "Gravity simulation - Variant 33"
    );

    window.setFramerateLimit(60);

	// Завантажуємо шрифт для інтерфейсу
    sf::Font font;

    if (!font.openFromFile("resources/fonts/arial.ttf"))
    {
        return 1;
    }

    // Створюємо початкову систему.
    resetSimulation();
    computeAccelerations();
    initialEnergy = calculateTotalEnergy();

	// Годинник для відліку часу кадру
    sf::Clock frameClock;
    double accumulator = 0.0;


    while (window.isOpen())
    {
        // Обробка подій
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }

            if (const auto* key =
                event->getIf<sf::Event::KeyPressed>())
            {
                if (key->code == sf::Keyboard::Key::Escape)
                {
                    window.close();
                }
                else if (key->code == sf::Keyboard::Key::Space)
                {
                    paused = !paused;
                }
                else if (key->code == sf::Keyboard::Key::R)
                {
                    resetSimulation();

                    computeAccelerations();

                    initialEnergy = calculateTotalEnergy();

                    accumulator = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::Up)
                {
                    gravitationalConstant =
                        std::min(
                            gravitationalConstant + 100.0,
                            5000.0
                        );

                    computeAccelerations();

                    initialEnergy = calculateTotalEnergy();

                    maximumRelativeEnergyError = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::Down)
                {
                    gravitationalConstant =
                        std::max(
                            gravitationalConstant - 100.0,
                            100.0
                        );

                    computeAccelerations();

                    initialEnergy = calculateTotalEnergy();

                    maximumRelativeEnergyError = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::Right)
                {
                    timeStep = std::min(
                        timeStep * 1.25,
                        0.01
                    );

                    accumulator = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::Left)
                {
                    timeStep = std::max(
                        timeStep / 1.25,
                        0.00025
                    );

                    accumulator = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::Num1)
                {
                    initialCondition = 1;

                    resetSimulation();
                    computeAccelerations();

                    initialEnergy = calculateTotalEnergy();
                    accumulator = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::Num2)
                {
                    initialCondition = 2;

                    resetSimulation();
                    computeAccelerations();

                    initialEnergy = calculateTotalEnergy();
                    accumulator = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::Num3)
                {
                    initialCondition = 3;

                    resetSimulation();
                    computeAccelerations();

                    initialEnergy = calculateTotalEnergy();
                    accumulator = 0.0;
                }
                else if (key->code == sf::Keyboard::Key::T)
                {
                    showTrajectories = !showTrajectories;
                }
            }
        }

        // Оновлення фізики

        const double frameTime =
            std::min(
                static_cast<double>(
                    frameClock.restart().asSeconds()
                    ),
                0.05
            );

        if (!paused)
        {
            accumulator += frameTime;
            while (accumulator >= timeStep)
            {
                updatePhysics(timeStep);
                accumulator -= timeStep;
            }
        }

        // Відображення

        window.clear(sf::Color(12, 15, 23));
        drawTrajectories(window);
        drawBodies(window);
        drawInterface(window, font);
        window.display();
    }

    return 0;
}
