#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_mainwindow.h"
#include "qinputdialog.h"


enum EdgeRenderMode
{
	NONE, 
	EDGE,
	HEAT,
	CUT,
	BULK,
	ORDER,
};

class QLabel;
class QPushButton;
class QCheckBox;
class QRadioButton;
class QGroupBox;
class RenderingWidget;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

private:
	void	CreateActions();
	void	CreateMenus();
	void	CreateToolBars();
	void	CreateStatusBar();
	void	CreateRenderGroup();	
	void	CreateEditGroup();
	void	CreateScaleGroup();

protected:
	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

signals:
	void	EdgeMode(int);

public slots:
	void	OpenFile();
	void	ShowMeshInfo(int npoint, int nedge);
	void	ShowCapturedVert(int id);
	void	ShowCapturedEdge(int id, double len);
	void	EdgeModeChange();
	void	ShowAbout();

	void	SetSlider();

private:
	Ui::MainWindowClass ui;

	// Basic
	QMenu				*menu_file_;
	QMenu				*menu_edit_;
	QMenu				*menu_help_;

	QToolBar			*toolbar_file_;
	QToolBar			*toolbar_edit_;
	QToolBar			*toolbar_basic_;
	QToolBar			*toolbar_fiber_;

	QAction				*action_new_;
	QAction				*action_open_;
	QAction				*action_save_;
	QAction				*action_saveas_;

	QAction				*action_aboutqt_;
	QAction				*action_about_;
	
	// Basic Operator Tool
	QAction				*action_loadmesh_;
	QAction				*action_background_;

	// Fiber
	QAction				*action_fiberprint_;
	QAction				*action_simplify_;
	QAction				*action_project_;

	// Render group
	QCheckBox			*checkbox_point_;
	QRadioButton		*radiobutton_edge_;
	QRadioButton		*radiobutton_heat_;
	QRadioButton		*radiobutton_cut_;
	QSlider				*slider_layer_;
	QRadioButton		*radiobutton_bulk_;
	QRadioButton		*radiobutton_order_;
	QSlider				*slider_order_;
	QRadioButton		*radiobutton_none_;
	QCheckBox			*checkbox_light_;
	QCheckBox			*checkbox_axes_;
	QGroupBox			*groupbox_render_;
	EdgeRenderMode		edge_render_;

	// Edit group
	QPushButton			*pushbutton_rotatexy_;
	QPushButton			*pushbutton_rotatexz_;
	QPushButton			*pushbutton_rotateyz_;
	QGroupBox			*groupbox_edit_;

	// Scale group
	QSlider				*slider_scale_;
	QGroupBox			*groupbox_scale_;

	// Information
	QLabel				*label_meshinfo_;
	QLabel				*label_operatorinfo_;
	QLabel				*label_modeinfo_;
	QLabel				*label_capture_;

	RenderingWidget		*renderingwidget_;
};

#endif // MAINWINDOW_H
