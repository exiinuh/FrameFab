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

	this->setWindowTitle("Fiber printing");

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
	layout_left->addWidget(groupbox_sep1_);
	layout_left->addStretch(1);

	QVBoxLayout *layout_right = new QVBoxLayout;
	layout_right->addWidget(groupbox_fiber_);
	layout_right->addWidget(groupbox_fiberpara_);
	layout_right->addWidget(groupbox_meshpara_);
	layout_right->addWidget(groupbox_debug_);
	layout_right->addWidget(groupbox_sep2_);
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

	action_about_ = new QAction(tr("About"), this);
	connect(action_about_, SIGNAL(triggered()), this, SLOT(ShowAbout()));
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

	menu_help_ = menuBar()->addMenu(tr("&Help"));
	menu_help_->setStatusTip(tr("Help"));
	menu_help_->addAction(action_about_);
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

	label_Dttol_	= new QLabel(QString("Stiff-offset tolerance: "), this);
	label_Drtol_	= new QLabel(QString("Stiff-offset tolerance: "), this);
	label_penalty_	= new QLabel(QString("ADMM penalty: "), this);
	label_pritol_	= new QLabel(QString("ADMM primal tolerance: "), this);
	label_dualtol_	= new QLabel(QString("ADMM dual tolerance: "), this);
	label_gamma_	= new QLabel(QString("Seq gamma: "), this);
	label_wl_		= new QLabel(QString("Seq Wl: "), this);
	label_wp_		= new QLabel(QString("Seq Wp: "), this);

	label_scale_	= new QLabel(QString("Scale: "), this);
	label_prolen_	= new QLabel(QString("Projection length: "), this);
}


void MainWindow::CreateSpinBoxes()
{
	spinbox_radius_ = new QDoubleSpinBox(this);
	spinbox_radius_->setFixedWidth(140);
	spinbox_radius_->setDecimals(2);
	spinbox_radius_->setRange(0.01, 0.5);
	spinbox_radius_->setValue(0.4);
	spinbox_radius_->setSingleStep(0.01);
	spinbox_radius_->setSuffix(" mm");

	spinbox_density_ = new QDoubleSpinBox(this);
	spinbox_density_->setFixedWidth(140);
	spinbox_density_->setDecimals(0);
	spinbox_density_->setRange(500, 5000);
	spinbox_density_->setValue(1210);
	spinbox_density_->setSingleStep(10);
	spinbox_density_->setSuffix(" *10^-12 T/mm^3");

	spinbox_g_ = new QDoubleSpinBox(this);
	spinbox_g_->setFixedWidth(140);
	spinbox_g_->setDecimals(2);
	spinbox_g_->setRange(-12000.00, -7000.00);
	spinbox_g_->setValue(-9806.33);
	spinbox_g_->setSingleStep(100);
	spinbox_g_->setSuffix(" mm/s^2");

	spinbox_youngsmodulus_ = new QDoubleSpinBox(this);
	spinbox_youngsmodulus_->setFixedWidth(140);
	spinbox_youngsmodulus_->setDecimals(0);
	spinbox_youngsmodulus_->setRange(0, 10000);
	spinbox_youngsmodulus_->setValue(1100);
	spinbox_youngsmodulus_->setSingleStep(1);
	spinbox_youngsmodulus_->setSuffix(" MPa");

	spinbox_shearmodulus_ = new QDoubleSpinBox(this);
	spinbox_shearmodulus_->setFixedWidth(140);
	spinbox_shearmodulus_->setDecimals(0);
	spinbox_shearmodulus_->setRange(0, 10000);
	spinbox_shearmodulus_->setValue(1032);
	spinbox_shearmodulus_->setSingleStep(1);
	spinbox_shearmodulus_->setSuffix(" MPa");

	spinbox_Dttol_ = new QDoubleSpinBox(this);
	spinbox_Dttol_->setFixedWidth(140);
	spinbox_Dttol_->setDecimals(4);
	spinbox_Dttol_->setRange(0, 1);
	spinbox_Dttol_->setValue(0.1);
	spinbox_Dttol_->setSingleStep(0.01);

	spinbox_Drtol_ = new QDoubleSpinBox(this);
	spinbox_Drtol_->setFixedWidth(140);
	spinbox_Drtol_->setDecimals(4);
	spinbox_Drtol_->setRange(0, 1);
	spinbox_Drtol_->setValue(10 * F_PI / 180);
	spinbox_Drtol_->setSingleStep(0.01);

	spinbox_penalty_ = new QDoubleSpinBox(this);
	spinbox_penalty_->setFixedWidth(140);
	spinbox_penalty_->setDecimals(2);
	spinbox_penalty_->setRange(0, 10000);
	spinbox_penalty_->setValue(1000);
	spinbox_penalty_->setSingleStep(1);

	spinbox_pritol_ = new QDoubleSpinBox(this);
	spinbox_pritol_->setFixedWidth(140);
	spinbox_pritol_->setDecimals(4);
	spinbox_pritol_->setRange(0, 1);
	spinbox_pritol_->setValue(0.001);
	spinbox_pritol_->setSingleStep(0.0001);

	spinbox_dualtol_ = new QDoubleSpinBox(this);
	spinbox_dualtol_->setFixedWidth(140);
	spinbox_dualtol_->setDecimals(4);
	spinbox_dualtol_->setRange(0, 1);
	spinbox_dualtol_->setValue(0.001);
	spinbox_dualtol_->setSingleStep(0.0001);

	spinbox_gamma_ = new QDoubleSpinBox(this);
	spinbox_gamma_->setFixedWidth(140);
	spinbox_gamma_->setDecimals(2);
	spinbox_gamma_->setRange(0, 100000);
	spinbox_gamma_->setValue(100);
	spinbox_gamma_->setSingleStep(1);

	spinbox_wl_ = new QDoubleSpinBox(this);
	spinbox_wl_->setFixedWidth(140);
	spinbox_wl_->setDecimals(2);
	spinbox_wl_->setRange(0, 10000);
	spinbox_wl_->setValue(10.0);
	spinbox_wl_->setSingleStep(1);

	spinbox_wp_ = new QDoubleSpinBox(this);
	spinbox_wp_->setFixedWidth(140);
	spinbox_wp_->setDecimals(2);
	spinbox_wp_->setRange(0, 10000);
	spinbox_wp_->setValue(1.0);
	spinbox_wp_->setSingleStep(1);

	spinbox_scale_ = new QDoubleSpinBox(this);
	spinbox_scale_->setFixedWidth(140);
	spinbox_scale_->setDecimals(1);
	spinbox_scale_->setRange(0, 1000);
	spinbox_scale_->setValue(1.0);
	spinbox_scale_->setSingleStep(1);
	connect(spinbox_scale_, SIGNAL(valueChanged(double)), this, SLOT(ShowScale(double)));
	connect(spinbox_scale_, SIGNAL(valueChanged(double)), renderingwidget_, SLOT(ScaleFrame(double)));

	spinbox_prolen_ = new QDoubleSpinBox(this);
	spinbox_prolen_->setFixedWidth(140);
	spinbox_prolen_->setDecimals(1);
	spinbox_prolen_->setRange(0, 1000);
	spinbox_prolen_->setValue(1.0);
	spinbox_prolen_->setSingleStep(1);
	connect(spinbox_prolen_, SIGNAL(valueChanged(double)), renderingwidget_, SLOT(ModifyProjection(double)));

	//pushbutton_scale_ = new QPushButton(tr("Scale"), this);
	//pushbutton_scale_->setFixedSize(80, 25);
	//connect(pushbutton_scale_, SIGNAL(clicked()), this, SLOT(CheckScale()));
	//connect(this, SIGNAL(ChangeScale(double)), renderingwidget_, SLOT(ScaleFrame(double)));
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
	slider_order_ = new QSlider(Qt::Horizontal, this);
	slider_order_->setFixedSize(80, 25);
	slider_order_->setMinimum(0);
	slider_order_->setMaximum(50);
	slider_order_->setSingleStep(1);
	connect(slider_order_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintOrder(int)));
	connect(renderingwidget_, SIGNAL(SetOrderSlider(int)), this, SLOT(SetOrderSlider(int)));
	connect(renderingwidget_, SIGNAL(SetMaxOrderSlider(int)), this, SLOT(SetMaxOrderSlider(int)));
}


void MainWindow::CreateRadioButtons()
{
	radiobutton_heat_ = new QRadioButton(tr("Heat"), this);
	connect(radiobutton_heat_, SIGNAL(clicked(bool)), this, SLOT(CheckEdgeMode()));

	radiobutton_bulk_ = new QRadioButton(tr("Bulk"), this);
	connect(radiobutton_bulk_, SIGNAL(clicked(bool)), this, SLOT(CheckEdgeMode()));

	radiobutton_order_ = new QRadioButton(tr("Order"), this);
	connect(radiobutton_order_, SIGNAL(clicked(bool)), this, SLOT(CheckEdgeMode()));

	radiobutton_none_ = new QRadioButton(tr("None"), this);
	radiobutton_none_->setVisible(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;
}


void MainWindow::CreatePushButtons()
{
	pushbutton_rotatexy_ = new QPushButton(tr("RotateXY"), this);
	pushbutton_rotatexz_ = new QPushButton(tr("RotateXZ"), this);
	pushbutton_rotateyz_ = new QPushButton(tr("RotateYZ"), this);
	pushbutton_rotatexy_->setFixedSize(80, 25);
	pushbutton_rotatexz_->setFixedSize(80, 25);
	pushbutton_rotateyz_->setFixedSize(80, 25);
	connect(pushbutton_rotatexy_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXY()));
	connect(pushbutton_rotatexz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXZ()));
	connect(pushbutton_rotateyz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateYZ()));

	pushbutton_nextedge_ = new QPushButton(tr("Next edge"), this);
	pushbutton_nextedge_->setFixedSize(80, 25);
	connect(pushbutton_nextedge_, SIGNAL(clicked()), this, SLOT(OrderStep()));

	//pushbutton_simplify_ = new QPushButton(tr("Simplify"), this);
	//pushbutton_simplify_->setFixedSize(80, 25);
	//connect(pushbutton_simplify_, SIGNAL(clicked()), renderingwidget_, SLOT(SimplifyFrame()));

	pushbutton_refine_ = new QPushButton(tr("Refine"), this);
	pushbutton_refine_->setFixedSize(80, 25);
	connect(pushbutton_refine_, SIGNAL(clicked()), renderingwidget_, SLOT(RefineFrame()));

	pushbutton_fiberprint_ = new QPushButton(tr("Fiber print"), this);
	pushbutton_fiberprint_->setFixedSize(140, 35);
	connect(pushbutton_fiberprint_, SIGNAL(clicked()), this, SLOT(GetFiberParas()));
	connect(this, 
		SIGNAL(SendFiberParas(
			double, double, double, 
			double, double, 
			double, double,
			double, double, double, 
			double, double, double)), 
		renderingwidget_, 
		SLOT(FiberPrintAnalysis(
			double, double, double, 
			double, double, 
			double, double, 
			double, double, double, 
			double, double, double)));

	pushbutton_project_ = new QPushButton(tr("Project"), this);
	pushbutton_project_->setFixedSize(140, 35);
	connect(pushbutton_project_, SIGNAL(clicked()), this, SLOT(GetProjectionParas()));
	connect(this, SIGNAL(SendProjectionParas(double)), renderingwidget_, SLOT(ProjectBound(double)));

	pushbutton_rightarrow_ = new QPushButton(tr(">>"), this);
	pushbutton_rightarrow_->setFlat(true);
	pushbutton_rightarrow_->setFixedSize(20, 20);
	connect(pushbutton_rightarrow_, SIGNAL(clicked()), this, SLOT(SwitchParaBox()));

	pushbutton_leftarrow_ = new QPushButton(tr("<<"), this);
	pushbutton_leftarrow_->setFlat(true);
	pushbutton_leftarrow_->setFixedSize(20, 20);
	connect(pushbutton_leftarrow_, SIGNAL(clicked()), this, SLOT(SwitchParaBox()));
}


void MainWindow::CreateToolButtons()
{
	toolbutton_choosebase_ = new QToolButton(this);
	toolbutton_choosebase_->setText(tr("Choose base"));
	toolbutton_choosebase_->setFixedSize(140, 35);
	connect(toolbutton_choosebase_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToChooseBase()));

	toolbutton_chooseceiling_ = new QToolButton(this);
	toolbutton_chooseceiling_->setText(tr("Choose ceiling"));
	toolbutton_chooseceiling_->setFixedSize(140, 35);
	connect(toolbutton_chooseceiling_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToChooseCeiling()));

	//toolbutton_addface_ = new QToolButton(this);
	//toolbutton_addface_->setText(tr("Set face"));
	//toolbutton_addface_->setMaximumSize(84, 50);
	//connect(toolbutton_addface_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToAddFace()));

	connect(renderingwidget_, SIGNAL(ChooseBasePressed(bool)), this, SLOT(ChooseBaseClicked(bool)));
	connect(renderingwidget_, SIGNAL(ChooseCeilingPressed(bool)), this, SLOT(ChooseCeilingClicked(bool)));
}


void MainWindow::CreateGroups()
{
	// render group
	connect(this, SIGNAL(ChangeEdgeMode(int)), renderingwidget_, SLOT(CheckEdgeMode(int)));

	groupbox_render_ = new QGroupBox(tr("Render"), this);
	groupbox_render_->setFlat(true);

	QVBoxLayout* render_layout = new QVBoxLayout(groupbox_render_);
	render_layout->addWidget(checkbox_point_);
	render_layout->addWidget(checkbox_light_);
	render_layout->addWidget(checkbox_axes_);

	// edge group
	groupbox_edge_ = new QGroupBox(tr("Edge"), this);
	groupbox_edge_->setCheckable(true);
	connect(groupbox_edge_, SIGNAL(clicked(bool)), this, SLOT(CheckEdgeMode()));

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
	//edit_layout->addWidget(toolbutton_addedge_);
	//edit_layout->addWidget(toolbutton_addface_);
	//edit_layout->addWidget(pushbutton_simplify_);
	edit_layout->addWidget(pushbutton_refine_);

	// separator group
	groupbox_sep1_ = new QGroupBox(this);
	groupbox_sep1_->setFlat(true);

	// fiber group
	groupbox_fiber_ = new QGroupBox(tr("Fiber"), this);
	groupbox_fiber_->setFlat(true);

	QVBoxLayout *fiber_layout = new QVBoxLayout(groupbox_fiber_);
	fiber_layout->addWidget(pushbutton_fiberprint_);
	fiber_layout->addWidget(toolbutton_choosebase_);
	fiber_layout->addWidget(toolbutton_chooseceiling_);
	fiber_layout->addWidget(pushbutton_project_);

	// parameter group
	groupbox_fiberpara_ = new QGroupBox(tr("Printing parameter"), this);
	groupbox_fiberpara_->setFlat(true);

	QVBoxLayout *fiberpara_layout = new QVBoxLayout(groupbox_fiberpara_);
	fiberpara_layout->addWidget(label_radius_);
	fiberpara_layout->addWidget(spinbox_radius_);
	fiberpara_layout->addWidget(label_density_);
	fiberpara_layout->addWidget(spinbox_density_);
	fiberpara_layout->addWidget(label_g_);
	fiberpara_layout->addWidget(spinbox_g_);
	fiberpara_layout->addWidget(label_youngsmodulus_);
	fiberpara_layout->addWidget(spinbox_youngsmodulus_);
	fiberpara_layout->addWidget(label_shearmodulus_);
	fiberpara_layout->addWidget(spinbox_shearmodulus_);

	groupbox_meshpara_ = new QGroupBox(tr("Mesh parameter"), this);
	groupbox_meshpara_->setFlat(true);

	QVBoxLayout *meshpara_layout = new QVBoxLayout(groupbox_meshpara_);
	meshpara_layout->addWidget(label_scale_);
	meshpara_layout->addWidget(spinbox_scale_);
	meshpara_layout->addWidget(label_prolen_);
	meshpara_layout->addWidget(spinbox_prolen_);
	meshpara_layout->addWidget(pushbutton_rightarrow_);

	// debug group
	groupbox_debug_ = new QGroupBox(tr("Debug"), this);
	groupbox_debug_->setFlat(true);

	QVBoxLayout *debug_layout = new QVBoxLayout(groupbox_debug_);
	debug_layout->addWidget(label_Dttol_);
	debug_layout->addWidget(spinbox_Dttol_);
	debug_layout->addWidget(label_Drtol_);
	debug_layout->addWidget(spinbox_Drtol_);
	debug_layout->addWidget(label_penalty_);
	debug_layout->addWidget(spinbox_penalty_);
	debug_layout->addWidget(label_pritol_);
	debug_layout->addWidget(spinbox_pritol_);
	debug_layout->addWidget(label_dualtol_);
	debug_layout->addWidget(spinbox_dualtol_);
	debug_layout->addWidget(label_gamma_);
	debug_layout->addWidget(spinbox_gamma_);
	debug_layout->addWidget(label_wl_);
	debug_layout->addWidget(spinbox_wl_);
	debug_layout->addWidget(label_wp_);
	debug_layout->addWidget(spinbox_wp_);

	debug_layout->addWidget(pushbutton_leftarrow_);

	// separator group
	groupbox_sep2_ = new QGroupBox(this);
	groupbox_sep2_->setFlat(true);
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


void MainWindow::ChooseBaseClicked(bool down)
{
	toolbutton_choosebase_->setDown(down);
}


void MainWindow::ChooseCeilingClicked(bool down)
{
	toolbutton_chooseceiling_->setDown(down);
}


void MainWindow::GetFiberParas()
{
	emit(SendFiberParas(
		spinbox_radius_->value(),
		spinbox_density_->value() * 1e-12,
		spinbox_g_->value(),
		spinbox_youngsmodulus_->value(),
		spinbox_shearmodulus_->value(),
		spinbox_Dttol_->value(),
		spinbox_Drtol_->value(),
		spinbox_penalty_->value(),
		spinbox_pritol_->value(),
		spinbox_dualtol_->value(),
		spinbox_gamma_->value(),
		spinbox_wl_->value(),
		spinbox_wp_->value()));
}


void MainWindow::GetProjectionParas()
{
	emit(SendProjectionParas(spinbox_prolen_->value()));
}


void MainWindow::CheckEdgeMode()
{
	if (groupbox_edge_->isChecked())
	{
		if (sender() == radiobutton_heat_)
		{
			if (edge_render_ != HEAT)
			{
				radiobutton_heat_->setChecked(true);
				edge_render_ = HEAT;
				emit(ChangeEdgeMode(HEAT));

				groupbox_orderdisplay_->setVisible(false);
				groupbox_edit_->setVisible(true);

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
				emit(ChangeEdgeMode(BULK));

				groupbox_orderdisplay_->setVisible(false);
				groupbox_edit_->setVisible(true);

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
				emit(ChangeEdgeMode(ORDER));

				groupbox_edit_->setVisible(false);
				groupbox_orderdisplay_->setVisible(true);

				return;
			}
		}

		radiobutton_none_->setChecked(true);
		edge_render_ = EDGE;
		emit(ChangeEdgeMode(EDGE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_edit_->setVisible(true);
	}
	else
	{
		radiobutton_none_->setChecked(true);
		edge_render_ = NONE;
		emit(ChangeEdgeMode(NONE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_edit_->setVisible(true);
	}
}


void MainWindow::SwitchParaBox()
{
	if (sender() == pushbutton_rightarrow_)
	{
		groupbox_fiberpara_->setVisible(false);
		groupbox_meshpara_->setVisible(false);
		groupbox_debug_->setVisible(true);
	}
	else
	{
		groupbox_debug_->setVisible(false);
		groupbox_fiberpara_->setVisible(true);
		groupbox_meshpara_->setVisible(true);
	}
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
		label_capture_->setVisible(true);
	}
	else
	{
		label_capture_->setVisible(false);
	}
}


void MainWindow::ShowCapturedEdge(int id, double len)
{
	if (id != -1)
	{
		label_capture_->setText(QString("Captured edge: %1  Length: %2").arg(id).arg(len));
		label_capture_->setVisible(true);
	}
	else
	{
		label_capture_->setVisible(false);
	}
}


void MainWindow::ShowScale(double scale)
{
	label_operatorinfo_->setText(QString("Scale: %1").arg(scale));
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

	label_operatorinfo_->setText(QString("Scale: 1.0"));

	groupbox_edge_->setChecked(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;

	groupbox_orderdisplay_->setVisible(false);
	groupbox_edit_->setVisible(true);

	groupbox_debug_->setVisible(false);
	groupbox_fiberpara_->setVisible(true);
	groupbox_meshpara_->setVisible(true);
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