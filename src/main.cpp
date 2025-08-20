#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <random>
using namespace std;

const unsigned int WIDTH = 1200;
const unsigned int HEIGHT = 900;
const float MIN_DIST = 30.0f;
const float SPEED = 0.07f; 

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> distX(0.0f, WIDTH);
std::uniform_real_distribution<float> distY(0.0f, HEIGHT);
std::uniform_real_distribution<float> distV(-2.0f, 2.0f);

struct Boid {
	sf::ConvexShape isoTriangle;
	sf::Vector2f pos;
	sf::Vector2f velocity;
	
	Boid() {
		isoTriangle.setPointCount(3);
		isoTriangle.setPoint(0, sf::Vector2f(0.f, -15.f)); 
		isoTriangle.setPoint(1, sf::Vector2f(-10.f, 10.f));  
		isoTriangle.setPoint(2, sf::Vector2f(10.f, 10.f));   
		isoTriangle.setFillColor(sf::Color(200, 200, 200));
	};

	
};

int main() {
    sf::RenderWindow window(sf::VideoMode({WIDTH, HEIGHT}), "Boid Flocking Sim");
	
	std::vector<Boid> boids;
	for (int i = 0; i < 100; i++) {
        Boid b;
        sf::Vector2f pos;
        bool validPos = false;

        while(!validPos) {
        	pos = {distX(gen), distY(gen)};
        	validPos = true;

			for(auto& other : boids) {
				sf::Vector2f diff = pos - other.pos;
				float dist = sqrt((diff.x * diff.x) + (diff.y * diff.y));
	
				if(dist < MIN_DIST) {
					validPos = false;
					break;
				}
			}	
        }

        b.pos = pos;
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        float vx = dist(gen);
        float vy = dist(gen);
        float length = sqrt(vx*vx + vy*vy);
       
        if (length > 0){
        	vx = (vx/length) * SPEED;
    		vy = (vy/length) * SPEED;	
        }	

		b.velocity = {vx, vy};
        b.isoTriangle.setPosition(b.pos);
        float angle = atan2(b.velocity.y, b.velocity.x) * 180.f / 3.14159265f;
        b.isoTriangle.setRotation(angle + 90.f);
        boids.push_back(b);
    }
    

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

	    window.clear(sf::Color(25, 25, 150));
	    for (auto& b : boids) {
		    b.pos += b.velocity;

		    if (b.pos.x > WIDTH) b.pos.x = 0;
		    if (b.pos.x < 0) b.pos.x = WIDTH;
		    if (b.pos.y > HEIGHT) b.pos.y = 0;
		    if (b.pos.y < 0) b.pos.y = HEIGHT;

		    b.isoTriangle.setPosition(b.pos);
	    	window.draw(b.isoTriangle);
	    }
	    window.display();
    }
}
