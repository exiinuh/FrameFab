#pragma once

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QRadioButton>
#include <QToolButton>
#include <QGroupBox>
#include <QSpinBox>
#include <QMessageBox>
#include <QKeyEvent>
#include <QSlider>
#include <QInputDialog>
#include <QLineEdit>

#include "GeneratedFiles\ui_mainwindow.h"
#include "renderingwidget.h"


enum EdgeRenderMode
{
	NONE,
	EDGE,
	HEAT,
	ORDER,
};

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
	void	CreateLabels();
	void	CreateSpinBoxes();
	void	CreateLineEdits();
	void	CreateCheckBoxes();
	void	CreateRadioButtons();
	void	CreatePushButtons();
	void	CreateToolButtons();
	void	CreateSliders();
	void	CreateGroups();
	void	CreateDialogs();

protected:
	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

signals:
	void	ChangeEdgeMode(int);
	void	SendFiberParas(double, double, double);
	void	SendDeformParas(double, double, double);
	void	SendProjectionParas(double);
	void	SendSaveOBJParas(QString);
	void	SendSavePWFParas(
				bool, bool,  
				bool, bool, 
				bool, int, int, 
				QString
			);
	void	SendExportParas(
				int, int, 
				QString, QString, QString
			);

public slots:
	void	OpenFile();

	void	ChooseBaseClicked(bool down);
	void	ChooseCeilingClicked(bool down);
	void	ChooseSubGClicked(bool down);

	/* mode = 1: normal fiber rountine; mode = 0: deformation calculation*/
	void	GetFiberParas();
	void	GetDeformParas();
	void	GetProjectionParas();
	void	GetSaveParas();
	void	GetExportParas();
	void	GetPath();

	void	CheckEdgeMode();
	void	SwitchParaBox();

	void	SetOrderSlider(int value);
	void	SetMaxOrderSlider(int max_value);

	void	SetMinLayer(int min_value);
	void	SetMaxLayer(int max_value);

	void	OpenSaveDialog();
	void	OpenExportDialog();

	void	ShowMeshInfo(int npoint, int nedge);
	void	ShowCapturedVert(int id, int degree);
	void	ShowCapturedEdge(int id, double len);
	void	ShowScale(double scale);
	void	ShowLayerInfo(int layer_id, int total_id);

	void	ShowAbout();
	void	ShowError(QString error_msg);

	void	Reset();

private:
	Ui::MainWindowClass ui;

	// Basic
	QMenu				*menu_file_;
	QMenu				*menu_display_;
	QMenu				*menu_help_;

	QAction				*action_new_;
	QAction				*action_open_;
	QAction				*action_save_;
	QAction				*action_import_;
	QAction				*action_export_;
	QAction				*action_exportrender_;

	QAction				*action_background_;

	QAction				*action_about_;
	
	// Labels
	QLabel				*label_meshinfo_;
	QLabel				*label_operatorinfo_;
	QLabel				*label_modeinfo_;
	QLabel				*label_capture_;
	QLabel				*label_layer_;

	QLabel				*label_wl_;
	QLabel				*label_wp_;
	QLabel				*label_wa_;

	QLabel				*label_scale_;
	QLabel				*label_prolen_;

	QLabel				*label_from1_;
	QLabel				*label_to1_;
	QLabel				*label_from2_;
	QLabel				*label_to2_;

	// Spinboxes
	QDoubleSpinBox		*spinbox_wl_;
	QDoubleSpinBox		*spinbox_wp_;
	QDoubleSpinBox		*spinbox_wa_;

	QDoubleSpinBox		*spinbox_scale_;
	QDoubleSpinBox		*spinbox_prolen_;

	QSpinBox			*spinbox_minlayer1_;
	QSpinBox			*spinbox_maxlayer1_;
	QSpinBox			*spinbox_minlayer2_;
	QSpinBox			*spinbox_maxlayer2_;

	// Lineedits
	QLineEdit			*lineedit_vertpath_;
	QLineEdit			*lineedit_linepath_;
	QLineEdit			*lineedit_renderpath_;
	QLineEdit			*lineedit_pwfpath_;

	// Checkboxes
	QCheckBox			*checkbox_point_;
	QCheckBox			*checkbox_light_;
	QCheckBox			*checkbox_axes_;
	QCheckBox			*checkbox_savevert_;
	QCheckBox			*checkbox_saveline_;
	QCheckBox			*checkbox_savepillar_;
	QCheckBox			*checkbox_saveceiling_;
	QCheckBox			*checkbox_savecut_;

	// Radiobuttons
	QRadioButton		*radiobutton_heat_;
	QRadioButton		*radiobutton_order_;
	QRadioButton		*radiobutton_none_;

	// Pushbuttons
	QPushButton			*pushbutton_rotatexy_;
	QPushButton			*pushbutton_rotatexz_;
	QPushButton			*pushbutton_rotateyz_;
	QPushButton			*pushbutton_lastedge_;
	QPushButton			*pushbutton_nextedge_;
	QPushButton			*pushbutton_lastlayer_;
	QPushButton			*pushbutton_nextlayer_;
	QPushButton			*pushbutton_fiberprint_;
	QPushButton			*pushbutton_project_;
	QPushButton			*pushbutton_rightarrow_;
	QPushButton			*pushbutton_leftarrow_;
	QPushButton			*pushbutton_save_;
	QPushButton			*pushbutton_export_;
	QPushButton			*pushbutton_exportvert_;
	QPushButton			*pushbutton_exportline_;
	QPushButton			*pushbutton_exportpath_;
	QPushButton			*pushbutton_deformation_;

	// Toolbuttons
	QToolButton			*toolbutton_choosebase_;
	QToolButton			*toolbutton_chooseb_ceiling_;
	QToolButton			*toolbutton_choosesubg_;

	// Sliders
	QSlider				*slider_order_;

	// Groupboxes
	QGroupBox			*groupbox_render_;
	QGroupBox			*groupbox_edge_;
	QGroupBox			*groupbox_orderdisplay_;
	QGroupBox			*groupbox_edit_;
	QGroupBox			*groupbox_fiber_;
	QGroupBox			*groupbox_meshpara_;
	QGroupBox			*groupbox_seqpara_;
	QGroupBox			*groupbox_debug_;
	QGroupBox			*groupbox_sep1_;
	QGroupBox			*groupbox_sep2_;
	QGroupBox			*groupbox_exportvert_;
	QGroupBox			*groupbox_exportline_;
	QGroupBox			*groupbox_exportlayer_;
	QGroupBox			*groupbox_exportpath_;
	QGroupBox			*groupbox_saveinfo_;
	QGroupBox			*groupbox_savelayer_;

	// Dialogs
	QDialog				*dialog_save_;
	QDialog				*dialog_export_;

	EdgeRenderMode		edge_render_;

	RenderingWidget		*renderingwidget_;
};

#endif // MAINWINDOW_H
