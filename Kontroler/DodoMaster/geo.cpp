
// int globalToLocal(long lon, long lat, int ind) {
//     double point[2];

//     point[0] = (double) lon / 10000000 - (double) leftColumnLon / 10000000;
//     point[1] = (double) lat / 10000000 - (double) leftColumnLat / 10000000;

//     point[0] *= scaleEW * visualizationScale;
//     point[1] *= scaleNS * visualizationScale;
//     double tmpPoint0 = point[0];

//     point[0] = point[0] * cos(-angle) - point[1] * sin(-angle);
//     point[1] = tmpPoint0 * sin(-angle) + point[1] * cos(-angle);

//     point[0] += visLeftColumnLon;
//     point[1] += visLeftColumnLat;

//     return (int) point[ind];
// }

// float getAngleVectors(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y) {
//     float vec1x = p1x - p2x;
//     float vec1y = p1y - p2y;

//     float vec2x = p3x - p2x;
//     float vec2y = p3y - p2y;

//     float result = acos((vec1x * vec2x + vec1y * vec2y) / sqrt(vec1x * vec1x + vec1y * vec1y) / sqrt(vec2x * vec2x + vec2y * vec2y));
    
//     if (vec1x * vec2y - vec1y * vec2x < 0) {
//         result *= -1;
//     }

//     return result;
// }

// float getAngle(int p1, int p2, int p3) {
//     return getAngleVectors(waypoints[p1][0], waypoints[p1][1], waypoints[p2][0], waypoints[p2][1], waypoints[p3][0], waypoints[p3][1]);
// }

// float getAnglePoints(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y) {
//     float a = distance(p1x, p1y, p2x, p2y);
//     float b = distance(p2x, p2y, p3x, p3y);
//     float c = distance(p1x, p1y, p3x, p3y);

//     float angleOut = acos((a * a + b * b - c * c) / (2 * a * b));

//     angleOut = pi - angleOut;

//     if ((p2x - p1x) * (p3y - p2y) - (p2y - p2y) * (p3x - p2x) > 0) {
//         angleOut *= -1;
//     }

//     return angleOut;
// }

// float distance(int p1x, int p1y, int p2x, int p2y) {
//     float x1 = p2x - p1x;
//     float y1 = p2y - p1y;
//     return sqrt(x1 * x1 + y1 * y1);
// }


// void calculateLine() {
//     int previousWaypoint = currentWaypoint - 1;
//     if (previousWaypoint < 0) {
//         previousWaypoint = waypointsNumber - 1;
//     }
//     int nextWaypoint = currentWaypoint + 1;
//     if (nextWaypoint >= waypointsNumber) {
//         nextWaypoint = 0;
//     }
//     //A - prev, B - next, C - current, D - tmp
//     float AcBcRatio = distance(waypoints[previousWaypoint][0], waypoints[previousWaypoint][1], waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]) / distance(waypoints[nextWaypoint][0], waypoints[nextWaypoint][1], waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
//     float AB = distance(waypoints[previousWaypoint][0], waypoints[previousWaypoint][1], waypoints[nextWaypoint][0], waypoints[nextWaypoint][1]);
//     float AD = AB / (1 + 1 / AcBcRatio);
//     float AdAbRatio = AD / AB;
//     float ADx = (waypoints[nextWaypoint][0] - waypoints[previousWaypoint][0]) * AdAbRatio;
//     float ADy = (waypoints[nextWaypoint][1] - waypoints[previousWaypoint][1]) * AdAbRatio;

//     float Dx = waypoints[previousWaypoint][0] + ADx;
//     float Dy = waypoints[previousWaypoint][1] + ADy;

//     lineA = -(waypoints[currentWaypoint][1] - Dy) / (waypoints[currentWaypoint][0] - Dx);
//     lineB = 1;
//     lineC = -Dy - lineA * Dx;

//     previousSide = 0;
//     if (lineA * waypoints[previousWaypoint][0] + lineB * waypoints[previousWaypoint][1] + lineC > 0) {
//         previousSide = 1;
//     }
//     int p1 = currentWaypoint;
//     int p2 = currentWaypoint + 1;
//     if (p2 >= waypointsNumber) {
//         p2 -= waypointsNumber;
//     }
//     int p3 = currentWaypoint + 2;
//     if (p3 >= waypointsNumber) {
//         p3 -= waypointsNumber;
//     }
//     for (int i = 3; i < 8; i++) {
//         if (p1 >= waypointsNumber) {
//             p1 = 0;
//         }
//         if (p2 >= waypointsNumber) {
//             p2 = 0;
//         }
//         if (p3 >= waypointsNumber) {
//             p3 = 0;
//         }
//         angles[i] = -pi + getAngle(p1, p2, p3);
//         p1++;
//         p2++;
//         p3++;
//     }

// }

// int mathAbs(int x) {
//     if (x >= 0) {
//         return x;
//     }
//     return x * -1;
// }


// float tansig(float x) {
//     return 2 / (1 + exp(-2 * x)) - 1;
// }