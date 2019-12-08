//
// Created by yue on 04.12.19.
//

#include <matching.hpp>

vector< vector<Vec3d> > getLibPieces() {
    // read whole json file
    string json;
    readFileToString("../../hardware/library.json", json);
    // convert to library of pieces
    vector< vector<Vec3d> > libPieces;
    Document document;
    document.Parse(json.c_str());
//    assert(document.IsArray());
//    assert(document[0].IsObject());
//    assert(document[0].HasMember("index"));
//    assert(document[0]["index"].IsInt());
    for (int i = 0; i < document.Size(); ++i) {
        vector<Vec3d> pieceToAppend;
        for (int j = 0; j < document[i]["vertices"].Size(); ++j)
            pieceToAppend.push_back(Vec3d(
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
            double depth = c.b * c.f / (contoursL[i][j].x - contoursR[i][j].x);
//            cout << depth << endl;
            // position L
            Mat positionLPixel = (Mat_<double>(3, 1)
                    << contoursL[i][j].x*depth, contoursL[i][j].y*depth, depth);
            Mat positionL = c.cameraLInv * positionLPixel;
//            cout << "positionL: " << Point3d(positionL) << endl;
            // position R
            Mat positionRPixel = (Mat_<double>(3, 1)
                    << contoursR[i][j].x*depth, contoursR[i][j].y*depth, depth);
            Mat positionR = c.cameraRInv * positionRPixel;
//            cout << "positionR: " << Point3d(positionR) << endl;
            if (abs(sum(positionL-positionR+c.T)[0]) > 5) return false;
            polylineToAppend.push_back(Point3d(positionL));
        }
//        cout << "contour left: " << contoursL[i] << endl;
//        cout << "contour right: " << contoursR[i] << endl;
        realPolylines.push_back(polylineToAppend);
    }

    return true;
}

int match(vector<Point3d> polyline) {
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
    Vec3d xAxis; // choose farthest vec as x-axis
    Point3d center = getCenter(polyline);
    for (auto p : polyline) {
        Vec3d vt = p - center;
        if (getLength(vt) > maxDis) {
            maxDis = getLength(vt);
            xAxis = vt;
        }
    }
    Pose pose(center, unitize(xAxis), zAxis);
    return pose;
}

Mat drawPose(Mat img, Pose pose, double length) {
    Mat originReal = pose.origin();
    Mat xReal = pose.origin() + pose.xAxis() * length;
    Mat yReal = pose.origin() + pose.yAxis() * length;
    Mat zReal = pose.origin() + pose.zAxis() * length;
    Point originPix = vec2point((Mat)(c.cameraL*originReal));
    Point xPix = vec2point((Mat)(c.cameraL*xReal));
    Point yPix = vec2point((Mat)(c.cameraL*yReal));
    Point zPix = vec2point((Mat)(c.cameraL*zReal));
    line(img, originPix, xPix, Scalar(0, 0, 255), 2);
    line(img, originPix, yPix, Scalar(0, 255, 0), 2);
    line(img, originPix, zPix, Scalar(255, 0, 0), 2);
    return img;
}

Point vec2point(Mat v) {
    Point p(v.at<double>(0, 0)/v.at<double>(2, 0), v.at<double>(1, 0)/v.at<double>(2, 0));
    return p;
}

Vec3d unitize(Vec3d v) {
    Vec3d unitVec;
    normalize(v, unitVec, 1, 0, NORM_L2);
    return unitVec;
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
