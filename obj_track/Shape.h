#ifndef SHAPE_H
#define SHAPE_H

struct Shape {
	int maxX;
	int maxY;
	int minX;
	int minY;
	double whitePercentage;

	int getArea() {
		return (maxX - minX) * (maxY - minY);
	}

	int getMidX() {
		return (maxX + minX) / 2;
	}
	int getMidY() {
		return (maxY + minY) / 2;
	}

	float getAspectRatio() {
		return (maxY - minY) / (maxX - minX);
	}

	float getPercentError() {
		return fabs(ASPECT_RATIO - this->getAspectRatio()) / ASPECT_RATIO;
	}

	bool isBetter(Shape *shape) {
		return this->getArea() / this->getPercentError()
				> shape->getArea() / shape->getPercentError();
	}
};

#endif
