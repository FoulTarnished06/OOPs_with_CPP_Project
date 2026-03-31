#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>

struct Vector2D {
    float x, y;

    Vector2D(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}

    Vector2D operator+(const Vector2D& v) const { return Vector2D(x + v.x, y + v.y); }
    Vector2D operator-(const Vector2D& v) const { return Vector2D(x - v.x, y - v.y); }
    Vector2D operator*(float scalar) const { return Vector2D(x * scalar, y * scalar); }
    Vector2D& operator+=(const Vector2D& v) { x += v.x; y += v.y; return *this; }

    float dot(const Vector2D& v) const { return x * v.x + y * v.y; }
    float magnitude() const { return std::sqrt(x * x + y * y); }
    
    Vector2D normalize() const {
        float mag = magnitude();
        if (mag == 0) return Vector2D(0, 0);
        return Vector2D(x / mag, y / mag);
    }
};

class RigidBody {
private:
    Vector2D position;
    Vector2D velocity;
    Vector2D acceleration;
    float mass;
    float radius;
    float restitution;
    sf::Color color;
    sf::CircleShape shape;

public:
    RigidBody(Vector2D pos, float m, float r, sf::Color c) 
        : position(pos), velocity(0, 0), acceleration(0, 0), mass(m), radius(r), restitution(0.8f), color(c) {
        shape.setRadius(radius);
        shape.setFillColor(color);
        shape.setOrigin(radius, radius); 
    }

    void applyForce(const Vector2D& force) {
        acceleration += force * (1.0f / mass);
    }

    void update(float dt) {
        velocity += acceleration * dt;
        position += velocity * dt;
        acceleration = Vector2D(0, 0); 
        shape.setPosition(position.x, position.y);
    }


    Vector2D getPosition() const { return position; }
    void setPosition(const Vector2D& pos) { position = pos; }
    Vector2D getVelocity() const { return velocity; }
    void setVelocity(const Vector2D& vel) { velocity = vel; }
    float getRadius() const { return radius; }
    float getMass() const { return mass; }
    float getRestitution() const { return restitution; }
    sf::CircleShape getShape() const { return shape; }
};

class PhysicsWorld {
private:
    std::vector<RigidBody> bodies;
    Vector2D gravity;
    float width, height;

    void checkBoundaries(RigidBody& body) {
        Vector2D pos = body.getPosition();
        Vector2D vel = body.getVelocity();
        float radius = body.getRadius();
        float restitution = body.getRestitution();

        if (pos.x - radius < 0) {
            pos.x = radius;
            vel.x = -vel.x * restitution;
        } else if (pos.x + radius > width) {
            pos.x = width - radius;
            vel.x = -vel.x * restitution;
        }

        if (pos.y - radius < 0) {
            pos.y = radius;
            vel.y = -vel.y * restitution;
        } else if (pos.y + radius > height) {
            pos.y = height - radius;
            vel.y = -vel.y * restitution;
        }

        body.setPosition(pos);
        body.setVelocity(vel);
    }

    void resolveCollisions() {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                RigidBody& b1 = bodies[i];
                RigidBody& b2 = bodies[j];

                Vector2D delta = b1.getPosition() - b2.getPosition();
                float dist = delta.magnitude();
                float min_dist = b1.getRadius() + b2.getRadius();

                if (dist < min_dist && dist > 0) {
                    Vector2D normal = delta.normalize();
                    float penetrationDepth = min_dist - dist;
                    
                    Vector2D correction = normal * (penetrationDepth / 2.0f);
                    b1.setPosition(b1.getPosition() + correction);
                    b2.setPosition(b2.getPosition() - correction);

                    Vector2D relativeVelocity = b1.getVelocity() - b2.getVelocity();
                    float velAlongNormal = relativeVelocity.dot(normal);

                    if (velAlongNormal > 0) continue; 

                    float e = std::min(b1.getRestitution(), b2.getRestitution());
                    float j_impulse = -(1 + e) * velAlongNormal;
                    j_impulse /= (1 / b1.getMass() + 1 / b2.getMass());

                    Vector2D impulse = normal * j_impulse;
                    b1.setVelocity(b1.getVelocity() + impulse * (1 / b1.getMass()));
                    b2.setVelocity(b2.getVelocity() - impulse * (1 / b2.getMass()));
                }
            }
        }
    }

public:
    PhysicsWorld(float w, float h) : gravity(0.0f, 980.0f), width(w), height(h) {}

    void addBody(const RigidBody& body) {
        bodies.push_back(body);
    }

    void step(float dt) {
        for (auto& body : bodies) {
            body.applyForce(gravity * body.getMass());
        }

        resolveCollisions();

        for (auto& body : bodies) {
            checkBoundaries(body);
            body.update(dt);
        }
    }

    void draw(sf::RenderWindow& window) {
        for (const auto& body : bodies) {
            window.draw(body.getShape());
        }
    }
};

int main() {
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 600;
    
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "2D Physics Engine");
    window.setFramerateLimit(60);

    PhysicsWorld world(WINDOW_WIDTH, WINDOW_HEIGHT);

    world.addBody(RigidBody(Vector2D(200, 100), 10.0f, 30.0f, sf::Color::Red));
    world.addBody(RigidBody(Vector2D(250, 300), 15.0f, 40.0f, sf::Color::Blue));
    world.addBody(RigidBody(Vector2D(400, 50), 5.0f, 20.0f, sf::Color::Green));
    world.addBody(RigidBody(Vector2D(500, 200), 20.0f, 50.0f, sf::Color::Yellow));

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
    
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    world.addBody(RigidBody(
                        Vector2D(event.mouseButton.x, event.mouseButton.y), 
                        10.0f, 25.0f, sf::Color::Red
                    ));
                }
            }
        }

        float dt = clock.restart().asSeconds();
        
   
        if (dt > 0.016f) dt = 0.016f; 

        world.step(dt);

        window.clear(sf::Color(30, 30, 30));
        world.draw(window);
        window.display();
    }
    return 0;}

