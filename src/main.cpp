#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <random>
using namespace std;

const unsigned int WIDTH = 800;
const unsigned int HEIGHT = 600;
const float MIN_DIST = 30.0f;

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> distX(0.0f, WIDTH);
std::uniform_real_distribution<float> distY(0.0f, HEIGHT);
std::uniform_real_distribution<float> distV(-2.0f, 2.0f);

// Craig Reynolds Boid Implementation
struct Boid {
    sf::ConvexShape isoTriangle;
    sf::Vector2f pos;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;

    float maxSpeed = 0.3f;
    float maxForce = 0.015f;

    float separationWeight = 1.5f;
    float alignmentWeight = 1.0f;
    float cohesionWeight = 1.0f;
    float perceptionRadius = 50.0f;

    Boid() {
        isoTriangle.setPointCount(3);
        isoTriangle.setPoint(0, sf::Vector2f(0.f, -10.f));
        isoTriangle.setPoint(1, sf::Vector2f(-6.f, 6.f));
        isoTriangle.setPoint(2, sf::Vector2f(6.f, 6.f));
        isoTriangle.setFillColor(sf::Color(255, 255, 100));
    }

	// Returns vector's length
    static float magnitude(const sf::Vector2f& v) {
        return sqrt(v.x*v.x + v.y*v.y);
    }

	// Scale the vector to unit length while maintaining its direction
    static sf::Vector2f normalize(const sf::Vector2f& v) {
        float m = magnitude(v);
        return (m > 0) ? v / m : sf::Vector2f(0.f,0.f);
    }

	// Controls vector top speed 
    static void limit(sf::Vector2f& v, float max) {
        float m = magnitude(v);
        if (m > max) v = normalize(v) * max;
    }

	// Boid Seperation
    sf::Vector2f separation(const std::vector<Boid>& boids) {
        float desiredSeparation = 25.f;
        sf::Vector2f steer(0.f, 0.f);
        int count = 0;

        for (const auto& other : boids) {
            if (&other == this) continue;
            float dx = pos.x - other.pos.x;
            float dy = pos.y - other.pos.y;
            float distSq = dx*dx + dy*dy;

            if (distSq > 0 && distSq < desiredSeparation*desiredSeparation) {
                sf::Vector2f diff(dx, dy);
                diff = normalize(diff);
                diff /= sqrt(distSq);
                steer += diff;
                count++;
            }
        }

        if (count > 0) steer /= (float)count;
        if (magnitude(steer) > 0) {
            steer = normalize(steer) * maxSpeed;
            steer -= velocity;
            limit(steer, maxForce);
        }
        return steer;
    }

	// Boid Alignment
    sf::Vector2f alignment(const std::vector<Boid>& boids) {
        sf::Vector2f steer(0.f, 0.f);
        int count = 0;

        for (const auto& other : boids) {
            if (&other == this) continue;
            float dx = pos.x - other.pos.x;
            float dy = pos.y - other.pos.y;
            float distSq = dx*dx + dy*dy;

            if (distSq < perceptionRadius*perceptionRadius) {
                steer += other.velocity;
                count++;
            }
        }

        if (count > 0) {
            steer /= (float)count;
            steer = normalize(steer) * maxSpeed;
            steer -= velocity;
            limit(steer, maxForce);
        }
        return steer;
    }

	// Boid Cohesion
    sf::Vector2f cohesion(const std::vector<Boid>& boids) {
        sf::Vector2f center(0.f, 0.f);
        int count = 0;

        for (const auto& other : boids) {
            if (&other == this) continue;
            float dx = pos.x - other.pos.x;
            float dy = pos.y - other.pos.y;
            float distSq = dx*dx + dy*dy;

            if (distSq < perceptionRadius*perceptionRadius) {
                center += other.pos;
                count++;
            }
        }

        if (count > 0) {
            center /= (float)count;
            sf::Vector2f desired = center - pos;
            desired = normalize(desired) * maxSpeed;
            sf::Vector2f steer = desired - velocity;
            limit(steer, maxForce);
            return steer;
        }
        return sf::Vector2f(0.f, 0.f);
    }

    void applyBehaviors(const std::vector<Boid>& boids) {
        sf::Vector2f sep = separation(boids) * separationWeight;
        sf::Vector2f ali = alignment(boids) * alignmentWeight;
        sf::Vector2f coh = cohesion(boids) * cohesionWeight;

        acceleration += sep + ali + coh;
    }

    void update() {
        velocity += acceleration;
        limit(velocity, maxSpeed);
        pos += velocity;
        acceleration *= 0.f;
    }

    void edges() {
        if (pos.x > WIDTH) pos.x = 0;
        if (pos.x < 0) pos.x = WIDTH;
        if (pos.y > HEIGHT) pos.y = 0;
        if (pos.y < 0) pos.y = HEIGHT;
    }

    void draw(sf::RenderWindow& window) {
        float angle = atan2(velocity.y, velocity.x) * 180.f / 3.14159265f;
        isoTriangle.setRotation(angle + 90.f);
        isoTriangle.setPosition(pos);
        window.draw(isoTriangle);
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode({WIDTH, HEIGHT}), "Boid Flocking Sim");
    sf::Clock clock;

    std::vector<Boid> boids;
    for (int i = 0; i < 150; i++) {
        Boid b;
        b.pos = {distX(gen), distY(gen)};
        float vx = distV(gen), vy = distV(gen);
        sf::Vector2f vel(vx, vy);
        if (Boid::magnitude(vel) > 0) vel = Boid::normalize(vel) * b.maxSpeed;
        b.velocity = vel;
        boids.push_back(b);
    }

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        window.clear(sf::Color(0, 0, 0));

        for (auto& b : boids) {
            b.applyBehaviors(boids);
            b.update();
            b.edges();
            b.draw(window);
        }

        float fps = 1.f / clock.restart().asSeconds(); 
        window.setTitle("Boid Flocking Sim - FPS: " + std::to_string((int)fps));

        window.display();
    }
}
