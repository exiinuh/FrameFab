#include "mainwindow.h"

#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QMessageBox>
#include <QKeyEvent>
#include "renderingwidget.h"


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	ui.mainToolBar->setVisible(false);

	renderingwidget_ = new RenderingWidget(this);
//	setCentralWidget(renderingwidget_);

	setGeometry(200, 150, 1000, 700);
	
	CreateActions();
	CreateMenus();
	CreateLabels();
	CreateSpinBoxes();
	CreateCheckBoxes();
	CreateSliders();
	CreateRadioButtons();
	CreatePushButtons();
	CreateToolButtons();
	CreateGroups();

	connect(renderingwidget_, SIGNAL(Reset()), this, SLOT(Reset()));
	
	QVBoxLayout *layout_left = new QVBoxLayout;
	layout_left->addWidget(groupbox_render_);
	layout_left->addWidget(groupbox_edge_);
	layout_left->addWidget(groupbox_orderdisplay_);
	layout_left->addWidget(groupbox_edit_);
	layout_left->addWidget(groupbox_scale_);
	layout_left->addWidget(groupbox_separator_);
	layout_left->addStretch(1);

	QVBoxLayout *layout_right = new QVBoxLayout;
	layout_right->addWidget(groupbox_fiber_);
	layout_right->addWidget(groupbox_para_);
	layout_right->addStretch(1);

	QHBoxLayout *layout_main = new QHBoxLayout;
	layout_main->addLayout(layout_left);
	layout_main->addWidget(renderingwidget_);
	layout_main->setStretch(1, 1);
	layout_main->addLayout(layout_right);
	this->centralWidget()->setLayout(layout_main);
	
	Reset();
}


MainWindow::~MainWindow()
{
	delete renderingwidget_;
	renderingwidget_ = NULL;
}


void MainWindow::CreateActions()
{
	action_new_ = new QAction(QIcon(":/Resources/images/new.png"), tr("New"), this);
	action_new_->setShortcut(QKeySequence::New);
	action_new_->setStatusTip(tr("Create a new file"));

	action_open_ = new QAction(QIcon(":/MainWindow/Resources/images/open.png"), tr("Open..."), this);
	action_open_->setShortcuts(QKeySequence::Open);
	action_open_->setStatusTip(tr("Open an existing file"));
	connect(action_open_, SIGNAL(triggered()), renderingwidget_, SLOT(ReadFrame()));

	action_save_ = new QAction(QIcon(":/MainWindow/Resources/images/save.png"), tr("Save"), this);
	action_save_->setShortcuts(QKeySequence::Save);
	action_save_->setStatusTip(tr("Save the document to disk"));
	connect(action_save_, SIGNAL(triggered()), renderingwidget_, SLOT(WriteFrame()));

	action_exportpoints_ = new QAction(tr("Export points"), this);
	connect(action_exportpoints_, SIGNAL(triggered()), renderingwidget_, SLOT(ExportPoints()));

	action_exportlines_ = new QAction(tr("Export lines"), this);
	connect(action_exportlines_, SIGNAL(triggered()), renderingwidget_, SLOT(ExportLines()));

	action_background_ = new QAction(tr("Change background"), this);
	connect(action_background_, SIGNAL(triggered()), renderingwidget_, SLOT(SetBackground()));
}


void MainWindow::CreateMenus()
{
	menu_file_ = menuBar()->addMenu(tr("&File"));
	menu_file_->setStatusTip(tr("File menu"));
	menu_file_->addAction(action_new_);
	menu_file_->addAction(action_open_);
	menu_file_->addAction(action_save_);

	menu_file_->addSeparator();
	menu_file_->addAction(action_exportpoints_);
	menu_file_->addAction(action_exportlines_);

	menu_display_ = menuBar()->addMenu(tr("&Display"));
	menu_display_->setStatusTip(tr("Display settings"));
	menu_display_->addAction(action_background_);
}


void MainWindow::CreateLabels()
{
	label_meshinfo_ = new QLabel(QString("MeshInfo: p: %1 e: %2").arg(0).arg(0), this);
	label_meshinfo_->setAlignment(Qt::AlignCenter);
	label_meshinfo_->setMinimumSize(label_meshinfo_->sizeHint());

	label_operatorinfo_ = new QLabel(QString("Scale: 1.0"), this);
	label_operatorinfo_->setAlignment(Qt::AlignVCenter);
	
	label_modeinfo_ = new QLabel(this);

	label_capture_ = new QLabel(this);

	statusBar()->addWidget(label_meshinfo_);
	connect(renderingwidget_, SIGNAL(meshInfo(int, int)), this, SLOT(ShowMeshInfo(int, int)));
	
	statusBar()->addWidget(label_operatorinfo_);
	connect(renderingwidget_, SIGNAL(operatorInfo(QString)), label_operatorinfo_, SLOT(setText(QString)));

	statusBar()->addWidget(label_modeinfo_);
	connect(renderingwidget_, SIGNAL(modeInfo(QString)), label_modeinfo_, SLOT(setText(QString)));

	statusBar()->addWidget(label_capture_);
	connect(renderingwidget_, SIGNAL(CapturedVert(int, int)), this, SLOT(ShowCapturedVert(int, int)));
	connect(renderingwidget_, SIGNAL(CapturedEdge(int, double)), this, SLOT(ShowCapturedEdge(int, double)));


	label_radius_		= new QLabel(QString("Radius: "), this);
	label_density_		= new QLabel(QString("Density: "), this);
	label_g_			= new QLabel(QString("Gravity: "), this);
	label_youngsmodulus_= new QLabel(QString("Young's modulus: "), this);
	label_shearmodulus_ = new QLabel(QString("Shear modulus: "), this);

	label_penalty_	= new QLabel(QString("ADMM penalty: "), this);
	label_Dtol_		= new QLabel(QString("ADMM D tolerance: "), this);
	label_pritol_	= new QLabel(QString("ADMM primal tolerance: "), this);
	label_dualtol_	= new QLabel(QString("ADMM dual tolerance: "), this);
	label_alpha_	= new QLabel(QString("TSP alpha: "), this);
	label_beta_		= new QLabel(QString("TSP beta: "), this);
	label_gamma_	= new QLabel(QString("TSP gamma: "), this);
}


void MainWindow::CreateSpinBoxes()
{
	spinbox_radius_ = new QDoubleSpinBox(this);
	spinbox_radius_->setDecimals(2);
	spinbox_radius_->setRange(0.01, 0.5);
	spinbox_radius_->setValue(0.4);
	spinbox_radius_->setSingleStep(0.01);
	spinbox_radius_->setSuffix(" mm");

	spinbox_density_ = new QDoubleSpinBox(this);
	spinbox_density_->setDecimals(0);
	spinbox_density_->setRange(500, 5000);
	spinbox_density_->setValue(1210);
	spinbox_density_->setSingleStep(10);
	spinbox_density_->setSuffix(" *10^-12 Ton/mm^3");

	spinbox_g_ = new QDoubleSpinBox(this);
	spinbox_g_->setDecimals(2);
	spinbox_g_->setRange(-12000.00, -7000.00);
	spinbox_g_->setValue(-9806.33);
	spinbox_g_->setSingleStep(100);
	spinbox_g_->setSuffix(" mm/s^2");

	spinbox_youngsmodulus_ = new QDoubleSpinBox(this);
	spinbox_youngsmodulus_->setDecimals(0);
	spinbox_youngsmodulus_->setRange(0, 10000);
	spinbox_youngsmodulus_->setValue(1100);
	spinbox_youngsmodulus_->setSingleStep(1);
	spinbox_youngsmodulus_->setSuffix(" MPa");

	spinbox_shearmodulus_ = new QDoubleSpinBox(this);
	spinbox_shearmodulus_->setDecimals(0);
	spinbox_shearmodulus_->setRange(0, 10000);
	spinbox_shearmodulus_->setValue(1032);
	spinbox_shearmodulus_->setSingleStep(1);
	spinbox_shearmodulus_->setSuffix(" MPa");

	spinbox_penalty_ = new QDoubleSpinBox(this);
	spinbox_penalty_->setDecimals(2);
	spinbox_penalty_->setRange(0, 10000);
	spinbox_penalty_->setValue(100);
	spinbox_penalty_->setSingleStep(1);

	spinbox_Dtol_ = new QDoubleSpinBox(this);
	spinbox_Dtol_->setDecimals(4);
	spinbox_Dtol_->setRange(0, 1);
	spinbox_Dtol_->setValue(0.1);
	spinbox_Dtol_->setSingleStep(0.01);

	spinbox_pritol_ = new QDoubleSpinBox(this);
	spinbox_pritol_->setDecimals(4);
	spinbox_pritol_->setRange(0, 1);
	spinbox_pritol_->setValue(0.001);
	spinbox_pritol_->setSingleStep(0.0001);

	spinbox_dualtol_ = new QDoubleSpinBox(this);
	spinbox_dualtol_->setDecimals(4);
	spinbox_dualtol_->setRange(0, 1);
	spinbox_dualtol_->setValue(0.001);
	spinbox_dualtol_->setSingleStep(0.0001);

	spinbox_alpha_ = new QDoubleSpinBox(this);
	spinbox_alpha_->setDecimals(2);
	spinbox_alpha_->setRange(0, 100000);
	spinbox_alpha_->setValue(1);
	spinbox_alpha_->setSingleStep(1);

	spinbox_beta_ = new QDoubleSpinBox(this);
	spinbox_beta_->setDecimals(2);
	spinbox_beta_->setRange(0, 100000);
	spinbox_beta_->setValue(10000);
	spinbox_beta_->setSingleStep(1);

	spinbox_gamma_ = new QDoubleSpinBox(this);
	spinbox_gamma_->setDecimals(2);
	spinbox_gamma_->setRange(0, 100000);
	spinbox_gamma_->setValue(100);
	spinbox_gamma_->setSingleStep(1);
}


void MainWindow::CreateCheckBoxes()
{
	checkbox_point_ = new QCheckBox(tr("Point"), this);
	connect(checkbox_point_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawPoint(bool)));
	checkbox_point_->setChecked(true);

	checkbox_light_ = new QCheckBox(tr("Light"), this);
	connect(checkbox_light_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckLight(bool)));

	checkbox_axes_ = new QCheckBox(tr("Axes"), this);
	connect(checkbox_axes_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawAxes(bool)));
}


void MainWindow::CreateSliders()
{
	/*
	slider_layer_ = new QSlider(Qt::Horizontal, this);
	slider_layer_->setMinimum(0);
	slider_layer_->setMaximum(10);
	connect(slider_layer_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintLayer(int)));
	*/

	slider_order_ = new QSlider(Qt::Horizontal, this);
	slider_order_->setMinimum(0);
	slider_order_->setMaximum(50);
	slider_order_->setSingleStep(1);
	connect(slider_order_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintOrder(int)));
	connect(renderingwidget_, SIGNAL(SetOrderSlider(int)), this, SLOT(SetOrderSlider(int)));
	connect(renderingwidget_, SIGNAL(SetMaxOrderSlider(int)), this, SLOT(SetMaxOrderSlider(int)));

	slider_scale_ = new QSlider(Qt::Horizontal, this);
	slider_scale_->setMinimum(1);
	slider_scale_->setMaximum(20);
	slider_scale_->setValue(10);
	connect(slider_scale_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(ScaleFrame(int)));
	connect(slider_scale_, SIGNAL(valueChanged(int)), this, SLOT(ShowScale(int)));
}


void MainWindow::CreateRadioButtons()
{
	radiobutton_heat_ = new QRadioButton(tr("Heat"), this);
	connect(radiobutton_heat_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_bulk_ = new QRadioButton(tr("Bulk"), this);
	connect(radiobutton_bulk_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_order_ = new QRadioButton(tr("Order"), this);
	connect(radiobutton_order_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_none_ = new QRadioButton(tr("None"), this);
	radiobutton_none_->setVisible(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;
}


void MainWindow::CreatePushButtons()
{
	pushbutton_nextedge_ = new QPushButton(tr("Next edge"), this);
	connect(pushbutton_nextedge_, SIGNAL(clicked()), this, SLOT(OrderStep()));

	pushbutton_rotatexy_ = new QPushButton(tr("RotateXY"), this);
	pushbutton_rotatexz_ = new QPushButton(tr("RotateXZ"), this);
	pushbutton_rotateyz_ = new QPushButton(tr("RotateYZ"), this);
	connect(pushbutton_rotatexy_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXY()));
	connect(pushbutton_rotatexz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXZ()));
	connect(pushbutton_rotateyz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateYZ()));

	pushbutton_simplify_ = new QPushButton(tr("Simplify"), this);
	connect(pushbutton_simplify_, SIGNAL(clicked()), renderingwidget_, SLOT(SimplifyFrame()));

	pushbutton_refine_ = new QPushButton(tr("Refine"), this);
	connect(pushbutton_refine_, SIGNAL(clicked()), renderingwidget_, SLOT(RefineFrame()));

	pushbutton_fiberprint_ = new QPushButton(tr("Fiber print"), this);
	pushbutton_project_ = new QPushButton(tr("Project"), this);
	connect(pushbutton_fiberprint_, SIGNAL(clicked()), this, SLOT(GetParameters())); 
	connect(this, SIGNAL(SendParameters(double, double, double, double, double, double, double, 
		double, double, double, double, double)), renderingwidget_, SLOT(FiberPrintAnalysis(double, 
		double, double, double, double, double, double, double, double, double, double, double)));
	connect(pushbutton_project_, SIGNAL(clicked()), renderingwidget_, SLOT(ProjectBound()));
}


void MainWindow::CreateToolButtons()
{
	toolbutton_choosebound_ = new QToolButton(this);
	toolbutton_choosebound_->setText(tr("Choose frame\nboundary"));
	toolbutton_choosebound_->setMaximumSize(200, 150);
	connect(toolbutton_choosebound_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToChooseBound()));

	toolbutton_addedge_ = new QToolButton(this);
	toolbutton_addedge_->setText(tr("Insert\nedge"));
	toolbutton_addedge_->setMaximumSize(100, 50);
	connect(toolbutton_addedge_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToAddEdge()));

	toolbutton_addface_ = new QToolButton(this);
	toolbutton_addface_->setText(tr("Set face"));
	toolbutton_addface_->setMaximumSize(84, 50);
	connect(toolbutton_addface_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToAddFace()));

	connect(renderingwidget_, SIGNAL(ChooseBoundPressed(bool)), this, SLOT(ChooseBoundClicked(bool)));
	connect(renderingwidget_, SIGNAL(AddEdgePressed(bool)), this, SLOT(AddEdgeClicked(bool)));
	connect(renderingwidget_, SIGNAL(AddFacePressed(bool)), this, SLOT(AddFaceClicked(bool)));
}


void MainWindow::CreateGroups()
{
	// render group
	connect(this, SIGNAL(EdgeMode(int)), renderingwidget_, SLOT(CheckEdgeMode(int)));

	groupbox_render_ = new QGroupBox(tr("Render"), this);
	groupbox_render_->setFlat(true);

	QVBoxLayout* render_layout = new QVBoxLayout(groupbox_render_);
	render_layout->addWidget(checkbox_point_);
	render_layout->addWidget(checkbox_light_);
	render_layout->addWidget(checkbox_axes_);

	// edge group
	groupbox_edge_ = new QGroupBox(tr("Edge"), this);
	groupbox_edge_->setCheckable(true);
	connect(groupbox_edge_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	QVBoxLayout* edge_layout = new QVBoxLayout(groupbox_edge_);
	edge_layout->addWidget(radiobutton_heat_);
	edge_layout->addWidget(radiobutton_bulk_);
	edge_layout->addWidget(radiobutton_order_);

	edge_layout->addWidget(radiobutton_none_);

	// order display group
	groupbox_orderdisplay_ = new QGroupBox(tr("Display"), this);
	groupbox_orderdisplay_->setFlat(true);

	QVBoxLayout* orderdisplay_layout = new QVBoxLayout(groupbox_orderdisplay_);
	orderdisplay_layout->addWidget(slider_order_);
	orderdisplay_layout->addWidget(pushbutton_nextedge_);

	// edit group
	groupbox_edit_ = new QGroupBox(tr("Edit"), this);
	groupbox_edit_->setFlat(true);

	QVBoxLayout* edit_layout = new QVBoxLayout(groupbox_edit_);
	edit_layout->addWidget(pushbutton_rotatexy_);
	edit_layout->addWidget(pushbutton_rotatexz_);
	edit_layout->addWidget(pushbutton_rotateyz_);
	edit_layout->addWidget(toolbutton_addedge_);
	edit_layout->addWidget(toolbutton_addface_);
	edit_layout->addWidget(pushbutton_simplify_);
	edit_layout->addWidget(pushbutton_refine_);

	// scale group
	groupbox_scale_ = new QGroupBox(tr("Scale"), this);
	groupbox_scale_->setFlat(true);

	QVBoxLayout* scale_layout = new QVBoxLayout(groupbox_scale_);
	scale_layout->addWidget(slider_scale_);

	// separator group
	groupbox_separator_ = new QGroupBox(this);
	groupbox_separator_->setFlat(true);

	// fiber group
	groupbox_fiber_ = new QGroupBox(tr("Fiber"), this);

	QVBoxLayout *fiber_layout = new QVBoxLayout(groupbox_fiber_);
	fiber_layout->addWidget(pushbutton_fiberprint_);
	fiber_layout->addWidget(toolbutton_choosebound_);
	fiber_layout->addWidget(pushbutton_project_);

	// parameter group
	groupbox_para_ = new QGroupBox(tr("Parameter"), this);

	QVBoxLayout *para_layout = new QVBoxLayout(groupbox_para_);
	para_layout->addWidget(label_radius_);
	para_layout->addWidget(spinbox_radius_);
	para_layout->addWidget(label_density_);
	para_layout->addWidget(spinbox_density_);
	para_layout->addWidget(label_g_);
	para_layout->addWidget(spinbox_g_);
	para_layout->addWidget(label_youngsmodulus_);
	para_layout->addWidget(spinbox_youngsmodulus_);
	para_layout->addWidget(label_shearmodulus_);
	para_layout->addWidget(spinbox_shearmodulus_);

	para_layout->addWidget(label_penalty_);
	para_layout->addWidget(spinbox_penalty_);
	para_layout->addWidget(label_Dtol_);
	para_layout->addWidget(spinbox_Dtol_);
	para_layout->addWidget(label_pritol_);
	para_layout->addWidget(spinbox_pritol_);
	para_layout->addWidget(label_dualtol_);
	para_layout->addWidget(spinbox_dualtol_);

	para_layout->addWidget(label_alpha_);
	para_layout->addWidget(spinbox_alpha_);
	para_layout->addWidget(label_beta_);
	para_layout->addWidget(spinbox_beta_);
	para_layout->addWidget(label_gamma_);
	para_layout->addWidget(spinbox_gamma_);
}


void MainWindow::keyPressEvent(QKeyEvent *e)
{

}


void MainWindow::keyReleaseEvent(QKeyEvent *e)
{

}


void MainWindow::OpenFile()
{

}


void MainWindow::ChooseBoundClicked(bool down)
{
	toolbutton_choosebound_->setDown(down);
}


void MainWindow::AddEdgeClicked(bool down)
{
	toolbutton_addedge_->setDown(down);
}


void MainWindow::AddFaceClicked(bool down)
{
	toolbutton_addface_->setDown(down);
}


void MainWindow::GetParameters()
{
	emit(SendParameters(
		spinbox_radius_->value(),
		spinbox_density_->value(),
		spinbox_g_->value(),
		spinbox_youngsmodulus_->value(),
		spinbox_shearmodulus_->value(),
		spinbox_penalty_->value(),
		spinbox_Dtol_->value(),
		spinbox_pritol_->value(),
		spinbox_dualtol_->value(),
		spinbox_alpha_->value(),
		spinbox_beta_->value(),
		spinbox_gamma_->value()
		)
	);
}


void MainWindow::OrderStep()
{
	slider_order_->setValue(slider_order_->value() + 1);
}


void MainWindow::SetOrderSlider(int value)
{
	slider_order_->setValue(value);
}


void MainWindow::SetMaxOrderSlider(int max_value)
{
	slider_order_->setMaximum(max_value);
}


void MainWindow::ShowMeshInfo(int npoint, int nedge)
{
	label_meshinfo_->setText(QString("MeshInfo: p: %1 e: %2").arg(npoint).arg(nedge));
}


void MainWindow::ShowCapturedVert(int id, int degree)
{
	if (id != -1)
	{
		label_capture_->setText(QString("Captured vertex: %1  Degree: %2").arg(id).arg(degree));
	}
	else
	{
		label_capture_->setText(QString(""));
	}
}


void MainWindow::ShowCapturedEdge(int id, double len)
{
	if (id != -1)
	{
		label_capture_->setText(QString("Captured edge: %1  Length: %2").arg(id).arg(len));
	}
	else
	{
		label_capture_->setText(QString(""));
	}
}


void MainWindow::ShowScale(int size)
{
	double scale = size*1.0 / 10;
	label_operatorinfo_->setText(QString("Scale: %1").arg(scale*scale));
}


void MainWindow::EdgeModeChange()
{
	if (groupbox_edge_->isChecked())
	{
		if (sender() == radiobutton_heat_)
		{
			if (edge_render_ != HEAT)
			{
				radiobutton_heat_->setChecked(true);
				edge_render_ = HEAT;
				emit(EdgeMode(HEAT));

				groupbox_orderdisplay_->setVisible(false);
				groupbox_edit_->setVisible(true);
				groupbox_scale_->setVisible(true);

				return;
			}
		}
		else
		if (sender() == radiobutton_bulk_)
		{
			if (edge_render_ != BULK)
			{
				radiobutton_bulk_->setChecked(true);
				edge_render_ = BULK;
				emit(EdgeMode(BULK));

				groupbox_orderdisplay_->setVisible(false);
				groupbox_edit_->setVisible(true);
				groupbox_scale_->setVisible(true);

				return;
			}
		}
		else
		if (sender() == radiobutton_order_)
		{
			if (edge_render_ != ORDER)
			{
				radiobutton_order_->setChecked(true);
				edge_render_ = ORDER;
				emit(EdgeMode(ORDER));

				groupbox_edit_->setVisible(false);
				groupbox_scale_->setVisible(false);
				groupbox_orderdisplay_->setVisible(true);

				return;
			}
		}

		radiobutton_none_->setChecked(true);
		edge_render_ = EDGE;
		emit(EdgeMode(EDGE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_edit_->setVisible(true);
		groupbox_scale_->setVisible(true);
	}
	else
	{
		radiobutton_none_->setChecked(true);
		edge_render_ = NONE;
		emit(EdgeMode(NONE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_edit_->setVisible(true);
		groupbox_scale_->setVisible(true);
	}
}


void MainWindow::ShowAbout()
{
	QMessageBox::information(this, "About QtMeshFrame-1.0.1",

		QString("<h3>This MeshFrame provides some operations about *.obj files sunch as") +
		" IO, render with points , edges, triangles or textures and some interactions with mouse."
		" A fix light source is provided for you."
		"This is a basic and raw frame for handling meshes. The mesh is of half_edge struct.\n"
		"Please contact" "<font color=blue> wkcagd@mail.ustc.edu.cn<\font><font color=black>, Kang Wang if you has any questions.<\font><\h3>"
		,
		QMessageBox::Ok);
}


void MainWindow::Reset()
{
	//slider_layer_->setValue(0);
	slider_order_->setValue(0);
	slider_scale_->setValue(10);
	label_operatorinfo_->setText(QString("Scale: 1.0"));

	groupbox_edge_->setChecked(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;

	groupbox_orderdisplay_->setVisible(false);
	groupbox_edit_->setVisible(true);
	groupbox_scale_->setVisible(true);
}
/*
void MainWindow::SetSlider()
{
	bool ok;
	int max_range = QInputDialog::getInt(
					this, 
					tr("Maximum Layers"), 
					tr("Please input the maximum layers"), 
					slider_layer_->maximum(), 0, 100, 1, &ok);
	if (ok)
	{
		slider_layer_->setMaximum(max_range);

	}
}
*/