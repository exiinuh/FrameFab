#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_mainwindow.h"
#include "qinputdialog.h"

class QLabel;
class QPushButton;
class QCheckBox;
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
	void	CreateSliders();

protected:
	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

public slots:
	void	ShowMeshInfo(int npoint, int nedge);
	void	ShowCapturedVert(int id);
	void	ShowCapturedEdge(int id, double len);
	void	OpenFile();
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
	QAction				*action_rotatexy_;
	QAction				*action_rotatexz_;
	QAction				*action_rotateyz_;

	// Fiber
	QAction				*action_fiberprint_;
	QAction				*action_simplify_;
	QAction				*action_project_;
	QAction				*action_setslider_;
	QSlider				*slider_layer_;

	// Render RadioButtons
	QCheckBox			*checkbox_point_;
	QCheckBox			*checkbox_edge_;
	QCheckBox			*checkbox_heat_;
	QCheckBox			*checkbox_bulk_;
	QCheckBox			*checkbox_light_;
	QCheckBox			*checkbox_axes_;

	QGroupBox			*groupbox_render_;

	// Information
	QLabel				*label_meshinfo_;
	QLabel				*label_operatorinfo_;
	QLabel				*label_modeinfo_;
	QLabel				*label_capture_;

	RenderingWidget		*renderingwidget_;
};

#endif // MAINWINDOW_H
