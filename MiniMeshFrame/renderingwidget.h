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
	Normal,
	Choosebound,
	Manualedge,
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
	void	meshInfo(int, int);
	void	operatorInfo(QString);
	void	modeInfo(QString); 
	void	CapturedVert(int);
	void	CapturedEdge(int, double);

private:
	void	CoordinatesTransform(QPoint p, double *x, double *y, double *z);
	bool	CaptureVertex(QPoint mouse);
	bool	CaptureEdge(QPoint mouse);

	void	Render();
	void	SetLight();

public slots:
	void	SetBackground();
	void	ReadFrame();
	void	WriteFrame();

	void	CheckDrawPoint(bool bv);
	void	CheckDrawEdge(bool bv);
	void	CheckDrawHeat(bool bv);
	void	CheckLight(bool bv);
	void	CheckDrawAxes(bool bv);

private:
	void	DrawAxes(bool bv);
	void	DrawPoints(bool bv);
	void	DrawEdge(bool bv);
	void	DrawHeat(bool bv);

public slots:
	void	FiberPrintAnalysis();
	void	SimplifyFrame();
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
	bool			is_draw_point_;
	bool			is_draw_edge_;
	bool			is_draw_heat_;
	bool			has_lighting_;
	bool			is_draw_axes_;

	// Fiber
	FiberPrintPlugIn	*ptr_fiberprint_;
	OperationMode		op_mode_;
	vector<int>			bound_;
	vector<WF_vert*>	captured_verts_;
	int					captured_edge_;
	bool				is_simplified_;

private:
	
};

#endif // RENDERINGWIDGET_H
