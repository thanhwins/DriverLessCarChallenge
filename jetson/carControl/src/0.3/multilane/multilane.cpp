#include "multilane.h"


const int MIN_LANE_AREA = 40; // no . pixels
const double INF = 1e9;
const double MIN_ANG_DIFF = 50; // 20 degree
double MIN_X_DIFF = 40; // pixels
double MIN_Y_DIFF = 110; // pixels
double MIN_ACC_DIST = MIN_ANG_DIFF + MIN_X_DIFF * 3 + MIN_Y_DIFF * 5;

struct Vector {
    double x, y;
    Vector(double x = 0, double y = 0) : x(x), y(y){}
    double operator * (const Vector &that) const {
        return x * that.x + y * that.y;
    }
    double mod() const{
        return sqrt(x * x + y * y);
    }
    // lower angle difference between 2 vectors in degree
    double operator ^ (const Vector &that) const {
        double rad = acos(fabs((*this) * that) / this->mod() / that.mod());
        return rad * 180 / M_PI;
    }
};

struct Edge {
    cv::Point a, b;
    Edge(){}
    Edge(cv::Point a, cv::Point b) : a(a), b(b){}
};

struct Lane { // higher y -> nearer to eyes
    Edge h, l;
    Lane() {}
    Lane(Edge h, Edge l) : h(h), l(l){}
    void show() const {
        fprintf(stderr, "low: (%d, %d) -> (%d, %d)\n", this->l.a.x, this->l.a.y, this->l.b.x, this->l.b.y);
        fprintf(stderr, "hig: (%d, %d) -> (%d, %d)\n", this->h.a.x, this->h.a.y, this->h.b.x, this->h.b.y);
    }
};

bool cmpByY(const Lane &one, const Lane &two) {
    double oneHigMidY = 0.5 * (one.h.a.y + one.h.b.y);
    double twoHigMidY = 0.5 * (two.h.a.y + two.h.b.y);
    return (oneHigMidY > twoHigMidY);
}

double laneDist(const Lane &one, const Lane &two) {
    // assuming that one.l < two.l by cmpByY
    double oneLowMidY = 0.5 * (one.l.a.y + one.l.b.y);
    double twoHigMidY = 0.5 * (two.h.a.y + two.h.b.y);
    if (oneLowMidY < twoHigMidY) // overlap 
        return 100000.0;
    double xDiff = 0.5 * fabs(two.h.a.x + two.h.b.x - one.l.a.x - one.l.b.x);
    double yDiff = 0.5 * fabs(two.h.a.y + two.h.b.y - one.l.a.y - one.l.b.y);
    if (xDiff > MIN_X_DIFF || yDiff > MIN_Y_DIFF)
        return 200000.0;
    Vector oneLow(one.l.b.x - one.l.a.x, one.l.b.y - one.l.a.y);
    Vector twoHig(two.h.b.x - two.h.a.x, two.h.b.y - two.h.a.y);
    double angDiff = oneLow ^ twoHig;
    // printf("ang: %lf\n", angDiff);
    if (angDiff > MIN_ANG_DIFF)
        return 300000.0;
    return angDiff + xDiff * 3 + yDiff * 5;
}

Lane joinTwoLane(const Lane &one, const Lane &two) {
    return Lane(one.h, two.l);
}

void addLanes(std::vector<cv::Point> poly, std::vector<Lane> &lanes) {
    if (poly.size() < 4) return;
    Edge l, h;
    double lsumY = 1e9, hsumY = -1e9;
    for (size_t i = 0; i < poly.size(); ++i) {
        size_t j = (i + 1) % poly.size();
        double sumY = poly[i].y + poly[j].y;
        if (lsumY > sumY) {
            lsumY = sumY;
            l = Edge(poly[i], poly[j]);
        }
        if (hsumY < sumY) {
            hsumY = sumY;
            h = Edge(poly[j], poly[i]);
        }
    }
    lanes.push_back(Lane(h, l));
}

bool joinLanes(std::vector<Lane> &lanes, std::vector< std::vector<cv::Point> > &joins) {
    double bestDist = MIN_ACC_DIST;
    int iOne = -1, iTwo = -1;
    for (int i = 0; i < (int)lanes.size() - 1; ++i)
        for (int j = i + 1; j < (int)lanes.size(); ++j) {
            double curDist = laneDist(lanes[i], lanes[j]);
            if (bestDist > curDist) {
                bestDist = curDist;
                iOne = i;
                iTwo = j;
            }
        }
    if (iOne != -1) {
        std::vector<Lane> tmpLane;
        for (int i = 0; i < (int)lanes.size(); ++i)
            if (i != iOne && i != iTwo)
                tmpLane.push_back(lanes[i]);
        tmpLane.push_back(joinTwoLane(lanes[iOne], lanes[iTwo]));
        std::vector<cv::Point> tmp;
        tmp.push_back(lanes[iOne].l.a);
        tmp.push_back(lanes[iOne].l.b);
        tmp.push_back(lanes[iTwo].h.b);
        tmp.push_back(lanes[iTwo].h.a);
        joins.push_back(tmp);
        lanes = tmpLane;
        return true;
    }
    return false;
}

cv::Mat removeOutlier(const cv::Mat &org, std::vector< std::vector< cv::Point> > &polys, bool verbose) {
    cv::Mat img = org.clone();
    if (img.channels() == 3)
        cv::cvtColor(img, img, CV_BGR2GRAY);
    // Binarize image
    cv::threshold(img, img, 160, 255, CV_THRESH_BINARY);
    if (verbose)
        cv::imshow("binary", img);
    // Erode image -> remove small outliers
    int esize = 1;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
        cv::Size( 2*esize + 1, 2*esize+1 ),
        cv::Point( esize, esize ) );
    cv::erode(img, img, element);
    if (verbose)
        cv::imshow("erosion", img);
    // cv::dilate(img, img, element);
    // get contours
    std::vector< std::vector<cv::Point> > contours;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    // approximate contours as polygons
    polys.clear();
    for (size_t i = 0; i < contours.size(); ++i) {
        std::vector<cv::Point> p;
        cv::approxPolyDP(cv::Mat(contours[i]), p, 2, true);
        if (p.size() < 3) continue;
        if (cv::contourArea(p) < MIN_LANE_AREA) continue;
        polys.push_back(p); 
    }
    // Draw polygons
    cv::Mat rem = cv::Mat::zeros(img.size(), CV_8UC1);
    for (size_t i = 0; i < polys.size(); ++i)
        cv::drawContours(rem, polys, i, cv::Scalar(255), CV_FILLED);
    if (verbose)
        cv::imshow("remOutlier-result", rem);
    return rem;
}

cv::Mat keepLanes(const cv::Mat &org, bool verbose = false) {
    if (verbose)
        cv::imshow("orgKeepLanes", org);
    cv::Rect roi(0, 3 * org.rows / 4, org.cols, 1 * org.rows / 4);
    cv::Mat img = org(roi).clone();
    std::vector< std::vector< cv::Point> > polys;
    std::vector<Lane> lanes;
    cv::Mat rem = removeOutlier(img, polys, verbose);
    if (verbose)
        cv::imshow("remOutlier", rem);

    for (auto poly : polys)
        addLanes(poly, lanes);
    
    std::vector< std::vector< cv::Point> > joins;
    while (true) {
        sort(lanes.begin(), lanes.end(), cmpByY);
        if (!joinLanes(lanes, joins)) break;
    }

    cv::Mat draft(rem.size(), CV_8UC1);
    for (size_t i = 0; i < polys.size(); ++i)
        cv::drawContours(draft, polys, i, cv::Scalar(255), CV_FILLED);
    for (size_t i = 0; i < joins.size(); ++i)
        cv::drawContours(draft, joins, i, cv::Scalar(255), CV_FILLED);
    if (verbose)
      //  cv::imshow("lanes", draft);
    return draft;
}

cv::Mat twoRightMostLanes(const cv::Size &size, const cv::Mat &imgLane, cv::Point shift, bool right) {
    // keep 2 lanes with highest average x values
    //cv::imshow("lane", imgLane);
    std::vector< std::vector<cv::Point> > contours;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours(imgLane, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    if (contours.size() <= 1) {
        cv::Mat none = cv::Mat::zeros(size, CV_8UC1);
        return none;
    }
    int sign = 1; //left
    if (right) sign = -1; // right
    fprintf(stderr, "sign = %d\n", sign);
    double x1Hig = 1e4 * sign, x2Hig = 1e4 * sign;
    int id1 = -1, id2 = -1;
    for (int i = 0; i < (int)contours.size(); ++i) {
        if (cv::contourArea(contours[i]) < 20) continue;
        double xMaxY = -1e9, maxY = -1e9;
        for (auto p : contours[i])
            if (maxY < p.y) {
                maxY = p.y;
                xMaxY = p.x;
            }
        // bam phai: xMaxY > x1Hig ... else xMaxY > x2Hig
        // bam trai: xMaxY < x1Hig ... else xMaxY < x2Hig
        // Khong sua cac dau <, > khac
        if (xMaxY * sign < x1Hig * sign) {
            x2Hig = x1Hig;
            id2 = id1;
            x1Hig = xMaxY;
            id1 = i;
        }
        else
        if (xMaxY * sign < x2Hig * sign) {
            x2Hig = xMaxY;
            id2 = i;
        }
    }
    fprintf(stderr, "%lf %lf\n", x2Hig, x1Hig);

    if (id1 == -1 || id2 == -1) {
        cv::Mat none = cv::Mat::zeros(size, CV_8UC1);
        return none;
    }

    cv::Mat ret = cv::Mat::zeros(size, CV_8UC1);
    std::vector< std::vector< cv::Point> > tmp;
    tmp.push_back(contours[id1]);
    tmp.push_back(contours[id2]);
    printf("%u %u\n", tmp[0].size(), tmp[1].size());
    for (auto &c : tmp)
        for (auto &p : c)
            p.x += shift.x, p.y += shift.y;
    for (size_t i = 0; i < tmp.size(); ++i)
        cv::drawContours(ret, tmp, i, cv::Scalar(255), CV_FILLED);
    return ret;
}
