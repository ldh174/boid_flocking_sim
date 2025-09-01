#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <random>
#include <vector>
#include <limits>
#include <cmath>
using namespace std;

const unsigned int WIDTH = 1200;
const unsigned int HEIGHT = 900;	

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
    
        float alpha = 255.f;
        float maxSpeed = 0.55f;
        float maxForce = 0.005f;
        float separationWeight = 1.0f;
        float alignmentWeight = 1.5f;
        float cohesionWeight = 0.5f;
        float perceptionRadius = 50.0f;
        float speedMultiplier;
    
        Boid() {
            isoTriangle.setPointCount(3);
            isoTriangle.setPoint(0, sf::Vector2f(0.f, -16.f));
            isoTriangle.setPoint(1, sf::Vector2f(-12.f, 12.f));
            isoTriangle.setPoint(2, sf::Vector2f(12.f, 12.f));
        }
    
        static float magnitude(const sf::Vector2f& v) {
            return sqrt(v.x*v.x + v.y*v.y);
        }
    
        static sf::Vector2f normalize(const sf::Vector2f& v) {
            float m = magnitude(v);
            return (m > 0) ? v / m : sf::Vector2f(0.f,0.f);
        }

    static void limit(sf::Vector2f& v, float max) {
        float m = magnitude(v);
        if (m > max) v = normalize(v) * max;
    }

	// Boid Seperation
    sf::Vector2f separation(const std::vector<Boid>& boids) {
	    float desiredSeparation = 45.f;
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
	        steer = normalize(steer) * maxSpeed - velocity;
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
	        steer = normalize(steer) * maxSpeed - velocity;
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

	// Flee from predator
    sf::Vector2f flee(const sf::Vector2f& predatorPos) {
	    sf::Vector2f diff = pos - predatorPos;
	    float dist = magnitude(diff);
	    	
	    if (dist < 100.f) {
	        diff = normalize(diff) * maxSpeed;
	        sf::Vector2f steer = diff - velocity;
	        limit(steer, maxForce * 2.f);
	        return steer;
	    }
	    
	    return sf::Vector2f(0.f, 0.f);
	}

    void applyBehaviors(const std::vector<Boid>& boids, const sf::Vector2f& predatorPos) {
        sf::Vector2f sep = separation(boids) * separationWeight;
        sf::Vector2f ali = alignment(boids) * alignmentWeight;
        sf::Vector2f coh = cohesion(boids) * cohesionWeight;
        sf::Vector2f fleeForce = flee(predatorPos) * 2.0f;

        acceleration += sep + ali + coh + fleeForce;
    }

	void update(const sf::Vector2f& predatorPos) {
	    velocity += acceleration;
	
	    float distToPredator = Boid::magnitude(pos - predatorPos);
	    float dynamicScale = 1.0f;
	    if (distToPredator < 150.f) dynamicScale = 1.5f; 
	    else if (distToPredator > 500.f) dynamicScale = 0.5f; 
	
	    float finalMaxSpeed = maxSpeed * speedMultiplier * dynamicScale;
	    Boid::limit(velocity, finalMaxSpeed);
	
	    pos += velocity;
	    acceleration *= 0.f;
	
	    if (distToPredator < 50.f) {
	        alpha = 255.f; 
	    } else if (distToPredator > 500.f) {
	        alpha = 50.f; 
	    } else {
	        float t = (distToPredator - 150.f) / (500.f - 150.f);
	        alpha = 255.f - t * (255.f - 100.f);
	    }
	
	    if (alpha > 255.f) alpha = 255.f;
	    if (alpha < 50.f) alpha = 50.f;
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
	
	    sf::Color fillColor(255, 255, 255, static_cast<sf::Uint8>(alpha));
	    isoTriangle.setFillColor(fillColor);
	
	    window.draw(isoTriangle);
	}
};

struct Predator {
    sf::ConvexShape shape;
    sf::Vector2f pos;
    sf::Vector2f velocity;
    float maxSpeed = 0.7f;  
    float maxForce = 0.005f;    

    int targetIndex = -1;
    float targetTimer = 0.f; 

	Predator(float width, float height) {
        shape.setPointCount(3);
        shape.setPoint(0, sf::Vector2f(0.f, -16.f));
        shape.setPoint(1, sf::Vector2f(-12.f, 12.f));
        shape.setPoint(2, sf::Vector2f(12.f, 12.f));
        shape.setOrigin(0.f, 0.f);
        shape.setFillColor(sf::Color(255, 0, 0));

        pos = sf::Vector2f(rand() % (int)width, rand() % (int)height);
        velocity = sf::Vector2f(0.f, 0.f);
    }

	void update(std::vector<Boid>& boids) {
	    if (boids.empty()) return;

	    sf::Vector2f center(0.f, 0.f);
	    float totalWeight = 0.f;

	    for (const auto& b : boids) {
	        sf::Vector2f diff = b.pos - pos;
	        float dist = Boid::magnitude(diff);

	        if (dist < 300.f) { 
	            float weight = (300.f - dist) + 50.f * (Boid::magnitude(b.velocity) / b.maxSpeed);
	            center += b.pos * weight;
	            totalWeight += weight;
	        }
	    }

	    if (totalWeight <= 0.f) {
	        int targetIndex = rand() % boids.size();
	        center = boids[targetIndex].pos;
	    } else {
	        center /= totalWeight;
	    }

	    sf::Vector2f predictedPos = center; 

	    sf::Vector2f desired = predictedPos - pos;
	    if (Boid::magnitude(desired) > 0.f) desired = Boid::normalize(desired) * maxSpeed;

	    sf::Vector2f perp(-desired.y, desired.x);
	    float jitter = ((rand() % 100) / 100.f - 0.5f) * 0.1f * maxSpeed;
	    desired += Boid::normalize(perp) * jitter;

	    sf::Vector2f steer = desired - velocity;
	    if (Boid::magnitude(steer) > maxForce)
	        steer = Boid::normalize(steer) * maxForce;

	    velocity += steer;
	    Boid::limit(velocity, maxSpeed);
	    pos += velocity;

	    float angle = atan2(velocity.y, velocity.x) * 180.f / 3.14159265f;
	    shape.setRotation(angle + 90.f);
	    shape.setPosition(pos);
	}


    void draw(sf::RenderWindow& window) {
        window.draw(shape);
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode({WIDTH, HEIGHT}), "Boid Flocking Sim with Predator");
    sf::Clock clock;

    std::vector<Boid> boids;
    for (int i = 0; i < 100; i++) {
        Boid b;
        b.pos = {distX(gen), distY(gen)};
        float vx = distV(gen), vy = distV(gen);
        sf::Vector2f vel(vx, vy);
        if (Boid::magnitude(vel) > 0) vel = Boid::normalize(vel) * b.maxSpeed;
        b.velocity = vel;
        
        int type = rand() % 3;
        float baseSpeed = 0.f;
        switch (type) {
            case 0: baseSpeed = 0.5f; break;   
            case 1: baseSpeed = 0.85f; break;
            case 2: baseSpeed = 1.2f; break; 
        }

        b.speedMultiplier = baseSpeed + static_cast<float>(rand())/RAND_MAX * 0.1f;

        boids.push_back(b);
    }

    Predator predator(WIDTH, HEIGHT);
    predator.pos = {WIDTH / 2.f, HEIGHT / 2.f};
    predator.velocity = {0.f, 0.f};

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        window.clear(sf::Color(25, 25, 25));
		
        for (auto& b : boids) {
            b.applyBehaviors(boids, predator.pos);
            b.update(predator.pos);
            b.edges();
            b.draw(window);
        }

        predator.update(boids);
        predator.draw(window);

        window.display();
    }
}
