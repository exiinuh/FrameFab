#pragma once
#include "Polyface.h"

class Triangle : public Polyface
{
public:
	Triangle()
	{
		vert_list_.push_back(point(1, 0, 0));
		vert_list_.push_back(point(0, 1, 0));
		vert_list_.push_back(point(0, 0, 1));
		Normal_();
	}

	Triangle(point v0, point v1, point v2)
	{
		vert_list_.push_back(v0);
		vert_list_.push_back(v1);
		vert_list_.push_back(v2);
		Normal_();
	}

	Triangle(GeoV3 v0, GeoV3 v1, GeoV3 v2)
	{
		vert_list_.push_back(Trans(v0));
		vert_list_.push_back(Trans(v1));
		vert_list_.push_back(Trans(v2));
		Normal_();
	}

	~Triangle() {}

public:
	void Print()
	{
		point _v0 = v0();
		point _v1 = v1();
		point _v2 = v2();

		std::cout << _v0.x() << ", " << _v0.y() << ", " << _v0.z() << std::endl;
		std::cout << _v1.x() << ", " << _v1.y() << ", " << _v1.z() << std::endl;
		std::cout << _v2.x() << ", " << _v2.y() << ", " << _v2.z() << std::endl;
	}

	void Render(WireFrame* ptr_frame, double alpha)
	{
		
		glBegin(GL_TRIANGLES);
		glColor4f(1.0, 1.0, 0, alpha);
		glNormal3fv(normal_);
		glVertex3fv(ptr_frame->Unify(v0()));
		glVertex3fv(ptr_frame->Unify(v1()));
		glVertex3fv(ptr_frame->Unify(v2()));
		glEnd();
	}

	void Add(point base)
	{
		for (int i = 0; i < vert_list_.size(); i++)
		{
			vert_list_[i] += base;
		}
	}
};

