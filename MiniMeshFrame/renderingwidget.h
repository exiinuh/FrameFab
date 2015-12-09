#pragma once

#ifndef RENDERINGWIDGET_H
#define RENDERINGWIDGET_H

#include <iostream>
#include <algorithm>

#include <QtWidgets/QMenu>
#include <QtWidgets/QAction>
#include <QGLWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <QTextCodec>

#include <gl\GLU.h>
#include <glut.h>

#include "mainwindow.h"
#include "ArcBall.h"
#include "globalFunctions.h"
#include "FiberPrint\FiberPrintPlugIn.h"
#include "WireFrame\WireFrame.h"


enum OperationMode
{
	NORMAL,
	CHOOSEBOUND,
	ADDEDGE,
	ADDFACE,
};


struct AddingFace
{
	vector<bool>		is_captured_vert_;
	vector<bool>		is_captured_edge_;
	vector<WF_vert*>	corner_points_;
};


class MainWindow;
class CArcBall;
class Mesh3D;

class RenderingWidget : public QGLWidget
{
	Q_OBJECT

public:
	RenderingWidget(QWidget *parent, MainWindow* mainwindow=0);
	~RenderingWidget();

public:
	void	InitDrawData();
	void	InitCapturedData();
	void	InitFiberData();

protected:
	void	initializeGL();
	void	resizeGL(int w, int h);
	void	paintGL();
	void	timerEvent(QTimerEvent *e);

	// mouse events
	void	mousePressEvent(QMouseEvent *e);
	void	mouseMoveEvent(QMouseEvent *e);
	void	mouseReleaseEvent(QMouseEvent *e);
	void	mouseDoubleClickEvent(QMouseEvent *e);
	void	wheelEvent(QWheelEvent *e);

public:
	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

signals:
	void	ChooseBoundPressed(bool);
	void	AddEdgePressed(bool);
	void	AddFacePressed(bool);

	void	SetOrderSlider(int);
	void	SetMaxOrderSlider(int);

	void	meshInfo(int, int);
	void	operatorInfo(QString);
	void	modeInfo(QString);
	void	CapturedVert(int, int);
	void	CapturedEdge(int, double);

	void	Reset();

private:
	void	CoordinatesTransform(QPoint p, double *x, double *y, double *z);
	bool	CaptureVertex(QPoint mouse);
	bool	CaptureEdge(QPoint mouse);
	bool	CaptureRing(QPoint mouse);

	void	Render();
	void	SetLight();

public slots:
	void	SetBackground();
	void	ReadFrame();
	void	WriteFrame();
	void	ScaleFrame(int size);

	void	ExportPoints();
	void	ExportLines();

	void	CheckDrawPoint(bool bv);
	void	CheckEdgeMode(int type);
	void	CheckLight(bool bv);
	void	CheckDrawAxes(bool bv);

	void	SwitchToNormal();
	void	SwitchToChooseBound();
	void	SwitchToAddEdge();
	void	SwitchToAddFace();

private:
	void	DrawAxes(bool bv);
	void	DrawPoints(bool bv);
	void	DrawEdge(bool bv);
	void	DrawHeat(bool bv);
	void	DrawBulk(bool bv);
	void	DrawOrder(bool bv);

public slots:
	void	FiberPrintAnalysis(double radius, double density, double g, double youngs_modulus, 
								double shear_modulus, double penalty, double D_tol, double pri_tol, 
								double dual_tol, double alpha, double beta, double gamma);
	void	PrintLayer(int layer);
	void	PrintOrder(int order);
	void	SimplifyFrame();
	void	RefineFrame();
	void	ProjectBound();

	void	RotateXY();
	void	RotateXZ();
	void	RotateYZ();

public:
	MainWindow		*ptr_mainwindow_;
	CArcBall		*ptr_arcball_;
	WireFrame		*ptr_frame_;

	// eye
	GLfloat			eye_distance_;
	point			eye_goal_;
	vec				eye_direction_;
	QPoint			current_position_;

	// Render information
	OperationMode	op_mode_;
	bool			is_draw_point_;
	bool			is_draw_edge_;
	bool			is_draw_heat_;
	bool			is_draw_bulk_;
	bool			is_draw_order_;
	bool			has_lighting_;
	bool			is_draw_axes_;

	// Fiber
	FiberPrintPlugIn	*ptr_fiberprint_;

	int					print_layer_;
	int					print_order_;

	bool				is_simplified_;

	vector<WF_vert*>	captured_verts_;
	WF_edge				*captured_edge_;
	AddingFace			capturing_face_;
	vector<int>			bound_;

	float				scale_;
};

#endif // RENDERINGWIDGET_H
