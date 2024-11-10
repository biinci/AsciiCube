#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

void gotoXY(int x, int y) {
    if (x >= 0 && y >= 0)
        std::cout << "\033[" << y << ";" << x << "H";
}

void clearScreen() {
    std::cout << "\033[2J\033[1;1H";
}
char getAsciiCharacter(float dot) {
    if (dot <= -0.9)
        return '@';
    if (dot <= -0.7)
        return '#';
    if (dot <= -0.4)
        return '+';
    if (dot <= -0.1)
        return '=';
    if (dot <= 0.1)
        return '-';
    if (dot <= 0.4)
        return ':';
    if (dot <= 0.7)
        return '.';
    if (dot <= 0.9)
        return '.';

    return '.';
}

float cubeSize = 30;
const int screenWidth = 80;
const int screenHeight = 24;

struct Point3D {
    float x, y, z;

    float magnitude() const { return std::sqrt(x * x + y * y + z * z); }

    Point3D normalized() const {
        float mag = magnitude();
        if (mag == 0) {
            return *this;
        }
        return Point3D{x / mag, y / mag, z / mag};
    }

    // Subtraction operator
    Point3D operator-(const Point3D& other) const {
        return Point3D{x - other.x, y - other.y, z - other.z};
    }

    // Addition operator
    Point3D operator+(const Point3D& other) const {
        return Point3D{x + other.x, y + other.y, z + other.z};
    }

    Point3D operator*(float scalar) const {
        return Point3D{x * scalar, y * scalar, z * scalar};
    }
};

std::ostream& operator<<(std::ostream& os, const Point3D& p) {
    os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    return os;
}

float zFactor = 0;
struct Point2D {
    float x, y;
    bool operator==(const Point2D& other) const {
        const float epsilon = 0.01f;
        return std::fabs(x - other.x) < epsilon &&
               std::fabs(y - other.y) < epsilon;
    }

    bool operator<(const Point2D& other) const {
        if (x == other.x)
            return y < other.y;
        return x < other.x;
    }
};

const Point3D cameraVector = {0, 0, 1};

Point3D RotateXY(const Point3D& p, float angle) {
    float cosA = cos(angle);
    float sinA = sin(angle);
    return {p.x * cosA - p.y * sinA, p.x * sinA + p.y * cosA, p.z};
}

Point3D RotateYZ(const Point3D& p, float angle) {
    float cosA = cos(angle);
    float sinA = sin(angle);
    return {p.x, p.y * cosA - p.z * sinA, p.y * sinA + p.z * cosA};
}

Point3D RotateXZ(const Point3D& p, float angle) {
    float cosA = cos(angle);
    float sinA = sin(angle);
    return {p.x * cosA - p.z * sinA, p.y, p.x * sinA + p.z * cosA};
}

Point2D projectTo2D(Point3D point3D) {
    float z = point3D.z + 50 + zFactor;
    if (z == 0) {
        z = 0.1f;
    }
    return {point3D.x / z * 40 + screenWidth / 2,
            point3D.y / z * 20 + screenHeight / 2};
}
struct Point2DHash {
    size_t operator()(const Point2D& p) const {
        size_t hash_x = std::hash<float>{}(p.x);
        size_t hash_y = std::hash<float>{}(p.y);
        return hash_x ^
               (hash_y << 1);  // bitwise XOR ve bit kaydırma ile birleşim
    }
};
Point3D crossProduct(const Point3D& v1, const Point3D& v2) {
    Point3D result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

float dotProduct(const Point3D& v1, const Point3D& v2) {
    float result = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    return result;
}
void setSurface(
    Point3D p1,
    Point3D p2,
    Point3D p3,
    Point3D p4,
    Point3D normalVector,
    std::unordered_map<Point2D, std::tuple<float, char>, Point2DHash>& toZ) {
    Point3D a = (p2 - p1).normalized();
    Point3D b = (p4 - p1).normalized();
    char ascii =
        getAsciiCharacter(dotProduct(cameraVector, normalVector.normalized()));

    for (float y = 0; y < cubeSize; y += 0.5) {
        Point3D initPos = b * y;
        for (float x = 0; x < cubeSize; x += 0.5) {
            Point3D pos = p1 + initPos + a * x;
            Point2D point = projectTo2D(pos);
            point = {std::floor(point.x), std::floor(point.y)};

            if (point.x >= 0 && point.x < screenWidth && point.y >= 0 &&
                point.y < screenHeight) {
                if (toZ.find(point) == toZ.end() ||
                    std::get<0>(toZ[point]) > pos.z) {
                    toZ[point] = std::make_pair(pos.z, ascii);
                }
            }
        }
    }
}

std::array<std::array<int, 4>, 6> surfaceIndexes = {{{0, 1, 2, 3},
                                                     {0, 1, 5, 4},
                                                     {0, 4, 7, 3},
                                                     {1, 5, 6, 2},
                                                     {4, 5, 6, 7},
                                                     {3, 2, 6, 7}}};

void drawCube(float alpha, float beta, float gamma) {
    clearScreen();

    std::array<Point3D, 8> points = {
        {{-cubeSize / 2, cubeSize / 2, -cubeSize / 2},
         {-cubeSize / 2, -cubeSize / 2, -cubeSize / 2},
         {cubeSize / 2, -cubeSize / 2, -cubeSize / 2},
         {cubeSize / 2, cubeSize / 2, -cubeSize / 2},
         {-cubeSize / 2, cubeSize / 2, cubeSize / 2},
         {-cubeSize / 2, -cubeSize / 2, cubeSize / 2},
         {cubeSize / 2, -cubeSize / 2, cubeSize / 2},
         {cubeSize / 2, cubeSize / 2, cubeSize / 2}}};
    for (auto& point : points) {
        point = RotateXY(point, alpha);
        point = RotateYZ(point, beta);
        point = RotateXZ(point, gamma);
    }
    std::array<Point3D, 6> normalVectors = {
        crossProduct(points[3] - points[0], points[1] - points[0]),
        crossProduct(points[1] - points[0], points[4] - points[0]),
        crossProduct(points[4] - points[0], points[3] - points[0]),
        crossProduct(points[2] - points[1], points[5] - points[1]),
        crossProduct(points[5] - points[4], points[7] - points[4]),
        crossProduct(points[3] - points[2], points[6] - points[2])};

    std::unordered_map<Point2D, std::tuple<float, char>, Point2DHash> toZ;

    for (int i = 0; i < 6; i += 1) {
        auto indexes = surfaceIndexes[i];
        setSurface(points[indexes[0]], points[indexes[1]], points[indexes[2]],
                   points[indexes[3]], normalVectors[i], toZ);
    }

    std::vector<std::string> frame(screenHeight, std::string(screenWidth, ' '));

    for (auto& keyValue : toZ) {
        frame[keyValue.first.y][keyValue.first.x] =
            std::get<1>(keyValue.second);
    }

    for (const auto& line : frame)
        std::cout << line << '\n';

    std::cout.flush();
}

int main() {
    float alpha = 0.2;
    float beta = 1.2;
    float gamma = 0;
    float factor = 0;
    while (true) {
        alpha += 0.1;
        gamma += 0.1 / 2;
        beta += 0.1 / 3;
        drawCube(alpha, beta, gamma);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
