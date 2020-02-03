/*
  GRANSAC from
  https://github.com/drsrinathsridhar/GRANSAC
*/

#pragma once


#include <mbzirc2020_task1_tasks/AbstractModel.hpp>

typedef std::array<GRANSAC::VPFloat, 3> Vector3VP;

class Point3D
	: public GRANSAC::AbstractParameter
{
public:
	Point3D(GRANSAC::VPFloat x, GRANSAC::VPFloat y, GRANSAC::VPFloat z)
	{
		m_Point3D[0] = x;
		m_Point3D[1] = y;
		m_Point3D[2] = z;
	};
  Point3D()
	{
          for (int i = 0; i < 3; ++i)
            m_Point3D[i] = 0;
	};

	Vector3VP m_Point3D;

  GRANSAC::VPFloat norm()
  {
    GRANSAC::VPFloat result = 0.0;
    for (int i = 0; i < 3; ++i)
      result += pow(m_Point3D[i], 2.0);
    return sqrt(result);
  }

  Point3D operator-(Point3D const &pt1)
  {
    Point3D pt;
    for (int i = 0; i < 3; ++i)
      pt.m_Point3D[i] = m_Point3D[i] - pt1.m_Point3D[i];
    return pt;
  }

  Point3D operator+(Point3D const &pt1)
  {
    Point3D pt;
    for (int i = 0; i < 3; ++i)
      pt.m_Point3D[i] = m_Point3D[i] + pt1.m_Point3D[i];
    return pt;
  }

  Point3D operator*(GRANSAC::VPFloat const &factor)
  {
    Point3D pt;
    for (int i = 0; i < 3; ++i)
      pt.m_Point3D[i] = m_Point3D[i] * factor;
    return pt;
  }

  Point3D cross(Point3D pt1){
    Point3D pt;
    pt.m_Point3D[0] = m_Point3D[1] * pt1.m_Point3D[2] - m_Point3D[2] * pt1.m_Point3D[1];
    pt.m_Point3D[1] = m_Point3D[2] * pt1.m_Point3D[0] - m_Point3D[0] * pt1.m_Point3D[2];
    pt.m_Point3D[2] = m_Point3D[0] * pt1.m_Point3D[1] - m_Point3D[1] * pt1.m_Point3D[0];
    return pt;
  }
};

class Line3DModel
	: public GRANSAC::AbstractModel<2>
{
protected:
	// Parametric form
  Point3D m_p1, m_p2;
	GRANSAC::VPFloat m_DistDenominator; // = |p1 - p2|. Stored for efficiency reasons

	virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
	{
		auto ExtPoint3D = std::dynamic_pointer_cast<Point3D>(Param);
		if (ExtPoint3D == nullptr)
			throw std::runtime_error("Line3DModel::ComputeDistanceMeasure() - Passed parameter are not of type Point3D.");

		// Return distance between passed "point" and this line
		// http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
                GRANSAC::VPFloat Numer = ((*ExtPoint3D - m_p1).cross(*ExtPoint3D - m_p2)).norm();
		GRANSAC::VPFloat Dist = Numer / m_DistDenominator;

		//// Debug
		//std::cout << "Point: " << ExtPoint3D->m_Point3D[0] << ", " << ExtPoint3D->m_Point3D[1] << std::endl;
		//std::cout << "Line: " << m_a << " x + " << m_b << " y + "  << m_c << std::endl;
		//std::cout << "Distance: " << Dist << std::endl << std::endl;

		return Dist;
	};

public:
	Line3DModel(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams)
	{
		Initialize(InputParams);
	};

	virtual void Initialize(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams) override
	{
		if (InputParams.size() != 2)
			throw std::runtime_error("Line3DModel - Number of input parameters does not match minimum number required for this model.");

		// Check for AbstractParamter types
		auto Point1 = std::dynamic_pointer_cast<Point3D>(InputParams[0]);
		auto Point2 = std::dynamic_pointer_cast<Point3D>(InputParams[1]);
		if (Point1 == nullptr || Point2 == nullptr)
			throw std::runtime_error("Line3DModel - InputParams type mismatch. It is not a Point3D.");

		std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());

		// Compute the line parameters
                m_p1 = Point3D(Point1->m_Point3D[0], Point1->m_Point3D[1], Point1->m_Point3D[2]);
                m_p2 = Point3D(Point2->m_Point3D[0], Point2->m_Point3D[1], Point2->m_Point3D[2]);

		m_DistDenominator = (m_p2 - m_p1).norm(); // Cache square root for efficiency
	};

	virtual std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>> Evaluate(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& EvaluateParams, GRANSAC::VPFloat Threshold)
	{
		std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
		int nTotalParams = EvaluateParams.size();
		int nInliers = 0;

		for (auto& Param : EvaluateParams)
		{
			if (ComputeDistanceMeasure(Param) < Threshold)
			{
				Inliers.push_back(Param);
				nInliers++;
			}
		}

		GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

		return std::make_pair(InlierFraction, Inliers);
	};

	virtual std::shared_ptr<GRANSAC::AbstractParameter> ComputeNearestPoint(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
  {
    // todo
    std::shared_ptr<Point3D> pt(new Point3D(0, 0, 0));

    return std::dynamic_pointer_cast<GRANSAC::AbstractParameter>(pt);

  };

	virtual GRANSAC::VPFloat getSlope() override
  {
    // todo
    return 0.0;
  };
};

