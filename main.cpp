#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>

struct Vec2 {
    float x, y;
    Vec2(float x = 0, float y = 0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    Vec2 operator/(float s) const { return Vec2(x / s, y / s); }
    Vec2 operator-() const { return Vec2(-x, -y); }
    float dot(const Vec2& v) const { return x * v.x + y * v.y; }
    float length() const { return std::sqrt(x * x + y * y); }
    Vec2 normalize() const { float len = length(); return len > 0 ? *this / len : *this; }
};

enum class ShapeType { Sphere, Box };

class Shape {
public:
    ShapeType type;
    virtual ~Shape() {}
};

class Sphere : public Shape {
public:
    float radius;
    Sphere(float r) : radius(r) { type = ShapeType::Sphere; }
};

class Box : public Shape {
public:
    Vec2 halfExtents;
    Box(Vec2 he) : halfExtents(he) { type = ShapeType::Box; }
};

class RigidBody {
public:
    Vec2 position, velocity, acceleration;
    float mass;
    Shape* shape;
    RigidBody(Vec2 pos, float m, Shape* s) : position(pos), velocity(0, 0), acceleration(0, 0), mass(m), shape(s) {}
    ~RigidBody() { delete shape; }
};

struct CollisionInfo {
    Vec2 normal;
    float depth;
};

bool checkSphereSphereCollision(Sphere* s1, Vec2 pos1, Sphere* s2, Vec2 pos2, CollisionInfo& info) {
    Vec2 delta = pos2 - pos1;
    float dist = delta.length();
    float sumRadii = s1->radius + s2->radius;
    if (dist < sumRadii) {
        info.normal = delta.normalize();
        info.depth = sumRadii - dist;
        return true;
    }
    return false;
}

bool checkSphereBoxCollision(Sphere* sphere, Vec2 spherePos, Box* box, Vec2 boxPos, CollisionInfo& info) {
    Vec2 closest = spherePos;
    Vec2 min = boxPos - box->halfExtents;
    Vec2 max = boxPos + box->halfExtents;
    if (closest.x < min.x) closest.x = min.x;
    if (closest.x > max.x) closest.x = max.x;
    if (closest.y < min.y) closest.y = min.y;
    if (closest.y > max.y) closest.y = max.y;
    Vec2 delta = spherePos - closest;
    float distSquared = delta.dot(delta);
    if (distSquared < sphere->radius * sphere->radius) {
        float dist = std::sqrt(distSquared);
        info.normal = dist > 0 ? delta / dist : Vec2(0, 1);
        info.depth = sphere->radius - dist;
        return true;
    }
    return false;
}

bool checkBoxBoxCollision(Box* box1, Vec2 pos1, Box* box2, Vec2 pos2, CollisionInfo& info) {
    Vec2 min1 = pos1 - box1->halfExtents, max1 = pos1 + box1->halfExtents;
    Vec2 min2 = pos2 - box2->halfExtents, max2 = pos2 + box2->halfExtents;
    if (max1.x > min2.x && min1.x < max2.x && max1.y > min2.y && min1.y < max2.y) {
        float overlapX = std::min(max1.x - min2.x, max2.x - min1.x);
        float overlapY = std::min(max1.y - min2.y, max2.y - min1.y);
        if (overlapX < overlapY) {
            info.normal = pos1.x < pos2.x ? Vec2(-1, 0) : Vec2(1, 0);
            info.depth = overlapX;
        } else {
            info.normal = pos1.y < pos2.y ? Vec2(0, -1) : Vec2(0, 1);
            info.depth = overlapY;
        }
        return true;
    }
    return false;
}

bool checkCollision(RigidBody* a, RigidBody* b, CollisionInfo& info) {
    if (a->shape->type == ShapeType::Sphere && b->shape->type == ShapeType::Sphere) {
        return checkSphereSphereCollision(static_cast<Sphere*>(a->shape), a->position,
                                        static_cast<Sphere*>(b->shape), b->position, info);
    }
    if (a->shape->type == ShapeType::Sphere && b->shape->type == ShapeType::Box) {
        return checkSphereBoxCollision(static_cast<Sphere*>(a->shape), a->position,
                                     static_cast<Box*>(b->shape), b->position, info);
    }
    if (a->shape->type == ShapeType::Box && b->shape->type == ShapeType::Sphere) {
        bool result = checkSphereBoxCollision(static_cast<Sphere*>(b->shape), b->position,
                            static_cast<Box*>(a->shape), a->position, info);
        if (result) info.normal = info.normal.operator-();
        return result;
    }
    if (a->shape->type == ShapeType::Box && b->shape->type == ShapeType::Box) {
        return checkBoxBoxCollision(static_cast<Box*>(a->shape), a->position,
                                  static_cast<Box*>(b->shape), b->position, info);
    }
    return false;
}

class PhysicsEngine {
public:
    std::vector<RigidBody*> bodies;
    Vec2 gravity = Vec2(0, -9.8f);

    void addBody(RigidBody* body) { bodies.push_back(body); }

    void update(float dt);
};

void PhysicsEngine::update(float dt) {
    for (auto body : bodies) body->acceleration = gravity;
    for (auto body : bodies) {
        body->velocity = body->velocity + body->acceleration * dt;
        body->position = body->position + body->velocity * dt;
    }
    
    // Handle collisions between bodies
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            RigidBody* a = bodies[i];
            RigidBody* b = bodies[j];
            CollisionInfo info;
            if (checkCollision(a, b, info)) {
                a->position = a->position - info.normal * (info.depth / 2);
                b->position = b->position + info.normal * (info.depth / 2);
                Vec2 relVel = b->velocity - a->velocity;
                float velAlongNormal = relVel.dot(info.normal);
                if (velAlongNormal > 0) continue;
                float e = 0.8f;
                float j = -(1 + e) * velAlongNormal / (1 / a->mass + 1 / b->mass);
                Vec2 impulse = info.normal * j;
                a->velocity = a->velocity - impulse / a->mass;
                b->velocity = b->velocity + impulse / b->mass;
            }
        }
    }

    // Handle ground collision
    for (auto body : bodies) {
        if (body->shape->type == ShapeType::Sphere) {
            Sphere* s = static_cast<Sphere*>(body->shape);
            if (body->position.y - s->radius < 0) {
                body->position.y = s->radius;
                body->velocity.y = -body->velocity.y * 0.8f;
            }
        } else if (body->shape->type == ShapeType::Box) {
            Box* b = static_cast<Box*>(body->shape);
            if (body->position.y - b->halfExtents.y < 0) {
                body->position.y = b->halfExtents.y;
                body->velocity.y = -body->velocity.y * 0.8f;
            }
        }
    }
}

class Renderer {
private:
    sf::RenderWindow window;
    float scale;

public:
    Renderer(int width, int height, float s) : window(sf::VideoMode(width, height), "Physics Engine"), scale(s) {}

    void render(const std::vector<RigidBody*>& bodies);
    bool isOpen() const { return window.isOpen(); }
    void pollEvent(PhysicsEngine& engine);
    Vec2 getMousePosition();
};

int main() {
    const float dt = 1.0f / 60.0f;
    PhysicsEngine engine;
    Renderer renderer(800, 600, 100.0f);

    // Add initial bodies
    engine.addBody(new RigidBody(Vec2(2.0f, 5.0f), 1.0f, new Sphere(0.5f)));
    engine.addBody(new RigidBody(Vec2(4.0f, 5.0f), 0.5f, new Sphere(0.3f)));
    engine.addBody(new RigidBody(Vec2(3.0f, 6.0f), 2.0f, new Box(Vec2(0.5f, 0.5f))));

    sf::Clock clock;
    while (renderer.isOpen()) {
        renderer.pollEvent(engine);
        engine.update(dt);
        renderer.render(engine.bodies);
        float elapsed = clock.getElapsedTime().asSeconds();
        if (elapsed < dt) sf::sleep(sf::seconds(dt - elapsed));
        clock.restart();
    }

    // Cleanup
    for (auto body : engine.bodies) delete body;
    return 0;
}