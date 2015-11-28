#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QToolButton>
#include <QSpinBox>
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
	void	CreateLabels();
	void	CreateSpinBoxes();
	void	CreateCheckBoxes();
	void	CreateRadioButtons();
	void	CreatePushButtons();
	void	CreateToolButtons();
	void	CreateSliders();
	void	CreateGroups();

protected:
	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

signals:
	void	EdgeMode(int);
	void	SendParameters(double, double, double, double, double, double, double,
							double, double, double, double, double);

public slots:
	void	OpenFile();

	void	ChooseBoundClicked(bool down);
	void	AddEdgeClicked(bool down);
	void	AddFaceClicked(bool down);

	void	GetParameters();

	void	OrderStep();
	void	SetOrderSlider(int value);
	void	SetMaxOrderSlider(int max_value);

	void	ShowMeshInfo(int npoint, int nedge);
	void	ShowCapturedVert(int id, int degree);
	void	ShowCapturedEdge(int id, double len);
	void	ShowScale(int size);
	void	EdgeModeChange();

	void	ShowAbout();

	void	Reset();

private:
	Ui::MainWindowClass ui;

	// Basic
	QMenu				*menu_file_;
	QMenu				*menu_edit_;
	QMenu				*menu_help_;

	QToolBar			*toolbar_file_;
	QToolBar			*toolbar_edit_;
	QToolBar			*toolbar_basic_;

	QAction				*action_new_;
	QAction				*action_open_;
	QAction				*action_save_;
	QAction				*action_saveas_;

	QAction				*action_aboutqt_;
	QAction				*action_about_;
	
	// Basic Operator Tool
	QAction				*action_loadmesh_;
	QAction				*action_background_;

	// Labels
	QLabel				*label_meshinfo_;
	QLabel				*label_operatorinfo_;
	QLabel				*label_modeinfo_;
	QLabel				*label_capture_;

	QLabel				*label_radius_;
	QLabel				*label_density_;
	QLabel				*label_g_;
	QLabel				*label_youngsmodulus_;
	QLabel				*label_shearmodulus_;

	QLabel				*label_penalty_;
	QLabel				*label_Dtol_;
	QLabel				*label_pritol_;
	QLabel				*label_dualtol_;

	QLabel				*label_alpha_;
	QLabel				*label_beta_;
	QLabel				*label_gamma_;

	// Spinboxes
	QDoubleSpinBox		*spinbox_radius_;
	QDoubleSpinBox		*spinbox_density_;
	QDoubleSpinBox		*spinbox_g_;
	QDoubleSpinBox		*spinbox_youngsmodulus_;
	QDoubleSpinBox		*spinbox_shearmodulus_;

	QDoubleSpinBox		*spinbox_penalty_;
	QDoubleSpinBox		*spinbox_Dtol_;
	QDoubleSpinBox		*spinbox_pritol_;
	QDoubleSpinBox		*spinbox_dualtol_;

	QDoubleSpinBox		*spinbox_alpha_;
	QDoubleSpinBox		*spinbox_beta_;
	QDoubleSpinBox		*spinbox_gamma_;

	// Checkboxes
	QCheckBox			*checkbox_point_;
	QCheckBox			*checkbox_light_;
	QCheckBox			*checkbox_axes_;

	// Radiobuttons
	QRadioButton		*radiobutton_heat_;
	QRadioButton		*radiobutton_cut_;
	QRadioButton		*radiobutton_bulk_;
	QRadioButton		*radiobutton_order_;
	QRadioButton		*radiobutton_none_;

	// Pushbuttons
	QPushButton			*pushbutton_nextedge_;
	QPushButton			*pushbutton_rotatexy_;
	QPushButton			*pushbutton_rotatexz_;
	QPushButton			*pushbutton_rotateyz_;
	QPushButton			*pushbutton_simplify_;
	QPushButton			*pushbutton_fiberprint_;
	QPushButton			*pushbutton_project_;

	// Toolbuttons
	QToolButton			*toolbutton_choosebound_;
	QToolButton			*toolbutton_addedge_;
	QToolButton			*toolbutton_addface_;

	// Sliders
	QSlider				*slider_layer_;
	QSlider				*slider_order_;
	QSlider				*slider_scale_;

	// Groupboxes
	QGroupBox			*groupbox_render_;
	QGroupBox			*groupbox_edge_;
	QGroupBox			*groupbox_separator_;
	QGroupBox			*groupbox_orderdisplay_;
	QGroupBox			*groupbox_edit_;
	QGroupBox			*groupbox_scale_;
	QGroupBox			*groupbox_fiber_;
	QGroupBox			*groupbox_para_;

	EdgeRenderMode		edge_render_;

	RenderingWidget		*renderingwidget_;
};

#endif // MAINWINDOW_H
