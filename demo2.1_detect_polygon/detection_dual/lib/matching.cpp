//
// Created by yue on 04.12.19.
//

#include <matching.hpp>

vector< vector<Point3d> > getLibPieces() {
    // read whole json file
    string json;
    readFileToString("../../hardware/library.json", json);
    // convert to library of pieces
    vector< vector<Point3d> > libPieces;
    Document document;
    document.Parse(json.c_str());
//    assert(document.IsArray());
//    assert(document[0].IsObject());
//    assert(document[0].HasMember("index"));
//    assert(document[0]["index"].IsInt());
    for (int i = 0; i < document.Size(); ++i) {
        vector<Point3d> pieceToAppend;
        for (int j = 0; j < document[i]["vertices"].Size(); ++j)
            pieceToAppend.push_back(Point3d(
                    document[i]["vertices"][j][0].GetDouble(),
                    document[i]["vertices"][j][1].GetDouble(),
                    document[i]["vertices"][j][2].GetDouble()));
        libPieces.push_back(pieceToAppend);
    }
    return libPieces;
}

bool validate(vector<vector<Point> > contoursL, vector<vector<Point> > contoursR, vector<vector<Point3d> > &realPolylines) {
    if (contoursL.size() != contoursR.size()) return false;

    for (int i = 0; i < contoursL.size(); ++i) {
        if (contoursL[i].size() != contoursR[i].size()) return false;
        if (contoursL[i].size() < 3) return false;

        vector<Point3d> polylineToAppend;
        for (int j = 0; j < contoursL[i].size(); ++j) {
            double depth = C.b * C.f / (contoursL[i][j].x - contoursR[i][j].x);
//            cout << depth << endl;
            // position L
            Mat positionLPixel = (Mat_<double>(3, 1)
                    << contoursL[i][j].x*depth, contoursL[i][j].y*depth, depth);
            Mat positionL = C.cameraLInv * positionLPixel;
//            cout << "positionL: " << Point3d(positionL) << endl;
            // position R
            Mat positionRPixel = (Mat_<double>(3, 1)
                    << contoursR[i][j].x*depth, contoursR[i][j].y*depth, depth);
            Mat positionR = C.cameraRInv * positionRPixel;
//            cout << "positionR: " << Point3d(positionR) << endl;
            if (abs(sum(positionL - positionR + C.T)[0]) > 5) return false;
            polylineToAppend.push_back(Point3d(positionL));
        }
//        cout << "contour left: " << contoursL[i] << endl;
//        cout << "contour right: " << contoursR[i] << endl;
        sortPolyline(polylineToAppend);
        realPolylines.push_back(polylineToAppend);
    }
    return true;
}

int match(vector<Point3d> polyline, Pose pose) {
    // transform polyline to piece coordinate
    vector<Point3d> polylineTransformed;
    for (int i = 0; i < polyline.size(); ++i) {
        polylineTransformed.push_back(transformCamera2Piece(polyline[i], pose));
    }
    for (int i = 0; i < LIBPIECES.size(); ++i) {
//        for (int shift = 0; shift < polyline.size(); ++shift) {
//            return 0;
//        }
        vector<Point3d> libPiece = LIBPIECES[i];
        // count doesn't match continue
        if (polylineTransformed.size() != libPiece.size()) continue;

        for (auto p : polylineTransformed) // log for debug
            cout << p;
        cout << endl;
        for (auto p : libPiece)
            cout << p;
        cout << endl; // log for debug

        // cost = sum of all the distance
        double cost = 0.;
        for (int j = 0; j < libPiece.size(); ++j) {
            cost += distance(polylineTransformed[j], libPiece[j]);
        }
        if (cost < 10.0) return i;
    }
    return -1;
}

Point3d getCenter(vector<Point3d> polyline) {
    Point3d center(0., 0., 0.);
    for (auto p : polyline) center += p;
    center.x /= polyline.size();
    center.y /= polyline.size();
    center.z /= polyline.size();
    return center;
}

Vec3d getNormal(vector<Point3d> polyline) {
    Point3d center = getCenter(polyline);
    Vec3d v0 = polyline[0] - center;
    Vec3d v1 = polyline[1] - center;
    return unitize(v0.cross(v1));
}

Pose getPose(vector<Point3d> polyline) {
    Vec3d zAxis = unitize(getNormal(polyline));
    double maxDis = 0.0;
    Point3d center = getCenter(polyline);
    Vec3d xAxis = polyline[0] - center; // polyline already sorted
//    Vec3d xAxis; // choose farthest vec as x-axis
//    for (auto p : polyline) {
//        Vec3d vt = p - center;
//        if (getLength(vt) > maxDis) {
//            maxDis = getLength(vt);
//            xAxis = vt;
//        }
//    }
    Pose pose(center, unitize(xAxis), zAxis);
    return pose;
}

Mat drawPose(Mat img, Pose pose, double length) {
    Mat originReal = pose.origin();
    Mat xReal = pose.origin() + pose.xAxis() * length;
    Mat yReal = pose.origin() + pose.yAxis() * length;
    Mat zReal = pose.origin() + pose.zAxis() * length;
    Point originPix = vec2point((Mat)(C.cameraL * originReal));
    Point xPix = vec2point((Mat)(C.cameraL * xReal));
    Point yPix = vec2point((Mat)(C.cameraL * yReal));
    Point zPix = vec2point((Mat)(C.cameraL * zReal));
    line(img, originPix, xPix, Scalar(0, 0, 255), 2);
    line(img, originPix, yPix, Scalar(0, 255, 0), 2);
    line(img, originPix, zPix, Scalar(255, 0, 0), 2);
    return img;
}

void sortPolyline(vector<Point3d> &polyline) {
    vector<Point3d> newPolyline;
    Point3d center = getCenter(polyline);
    int maxIndex = 0;
    for (int i = 0; i < polyline.size(); ++i) {
        double disTemp = distance(center, polyline[i]);
        if (disTemp > distance(center, polyline[maxIndex])) maxIndex = i;
    }
    for (int i = 0; i < polyline.size(); ++i) {
        newPolyline.push_back(polyline[(maxIndex+i)%polyline.size()]);
    }
    polyline = newPolyline;
}

Point vec2point(Mat v) {
    Point p(v.at<double>(0, 0)/v.at<double>(2, 0), v.at<double>(1, 0)/v.at<double>(2, 0));
    return p;
}

Point3d transformCamera2Piece(Point3d p, Pose piecePose) {
    Mat vp = (Mat_<double>(3, 1)
            << p.x, p.y, p.z);
    Mat vReselt = piecePose.R().t() * (vp - piecePose.origin());
    return Point3d(
            vReselt.at<double>(0, 0),
            vReselt.at<double>(1, 0),
            vReselt.at<double>(2, 0));
}

Vec3d unitize(Vec3d v) {
    Vec3d unitVec;
    normalize(v, unitVec, 1, 0, NORM_L2);
    return unitVec;
}

double distance(Point3d p1, Point3d p2) {
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

double getLength(Vec3d v) {
    return sqrt(v.dot(v));
}

bool readFileToString(string file_name, string& fileData) {
    ifstream file(file_name.c_str(),  std::ifstream::binary);
    if(file)
    {
        // Calculate the file's size, and allocate a buffer of that size.
        file.seekg(0, file.end);
        const int file_size = file.tellg();
        char* file_buf = new char [file_size+1];
        //make sure the end tag \0 of string.
        memset(file_buf, 0, file_size+1);

        // Read the entire file into the buffer.
        file.seekg(0, ios::beg);
        file.read(file_buf, file_size);

        if(file)
        {
            fileData.append(file_buf);
        }
        else
        {
            std::cout << "error: only " <<  file.gcount() << " could be read";
            fileData.append(file_buf);
            return false;
        }
        file.close();
        delete []file_buf;
    }
    else
    {
        return false;
    }
    return true;
}
