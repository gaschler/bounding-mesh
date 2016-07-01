//	Copyright (c) 2013, Andre Gaschler, Quirin Fischer
//	All rights reserved.
//	
//	Redistribution and use in source and binary forms, with or without modification,
//	are permitted provided that the following conditions are met:
//	
//	* Redistributions of source code must retain the above copyright notice, this
//	  list of conditions and the following disclaimer.
//	
//	* Redistributions in binary form must reproduce the above copyright notice, this
//	  list of conditions and the following disclaimer in the documentation and/or
//	  other materials provided with the distribution.
//	
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QGLWidget>
#include <QMessageBox>
#include <QMimeData>
#include <QUrl>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/SoQtGLWidget.h>

#include <boundingmesh.h>
#include "gui.h"

#include <iostream>
#include <sstream>
#include "../../thirdparty/optionparser.h"

#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/nodes/SoPolygonOffset.h>

int minimal_vertices = 50;
int number_decimate_many = 100;
int number_decimate_many_more = 5000;
boundingmesh::Real maximum_error = 1.0;

CustomLineEdit::CustomLineEdit(QWidget *parent)
	:QLineEdit(parent)
{
	select_on_mouse_press_ = false;
}

void CustomLineEdit::focusInEvent(QFocusEvent *e)
{
	QLineEdit::focusInEvent(e);
	if(text() == "")
	{
		setText(placeholderText());
		selectAll();
		select_on_mouse_press_ = true;
	}
}

void CustomLineEdit::mousePressEvent(QMouseEvent *me)
{
	QLineEdit::mousePressEvent(me);
	if(select_on_mouse_press_)
	{
		selectAll();
		select_on_mouse_press_ = false;
	}
}

void MainWindow::loadView()
{
	original_mesh = this->mesh;
	
	decimator = std::make_shared< boundingmesh::Decimator >();
	decimator->setDirection(direction);
	decimator->setMetric(metric);

	decimator->setMesh(*mesh);
	mesh = decimator->getMesh();
	if(!radio_decimated_invis->isChecked())
		viewer_mesh->replaceByMesh(mesh);
	if(!radio_original_invis->isChecked())
		viewer_mesh_original->replaceByMesh(original_mesh);

	SoGetBoundingBoxAction bounding_box_action( viewer->getViewportRegion() );
	bounding_box_action.apply( viewer_mesh_original->model_root);
	SbBox3f bbox = bounding_box_action.getBoundingBox();
	SbVec3f size = bbox.getSize();
	
	std::cout << "Bounding box size: " << size[0] << " " << size[1] << " " << size[2];
	float diagonal = sqrt(size[0]*size[0] + size[1]*size[1] + size[2]*size[2]);
	std::cout << " Bounding box diagonal: " << diagonal << std::endl;
	maximum_error = bbox.getSize().length() / 20 * bbox.getSize().length() / 20;
	std::cout << "Maximum error: " << maximum_error << std::endl;
	
	decimator->setMaximumError(maximum_error);
	using_error_restriction_ = true;

	button_decimate->setDisabled(false);
	button_decimate_many->setDisabled(false);
	button_decimate_many_more->setDisabled(false);
	button_decimate_vertices->setDisabled(false);
	button_decimate_error->setDisabled(false);
	line_desired_vertices->setText("");
	line_desired_error->setText("");

	reload();
	viewer->viewAll();
}

#include <QDir>
#include <QMessageBox>

bool MainWindow::openFile(const std::string &filename_in)
{
	mesh.reset(new boundingmesh::Mesh);
	boundingmesh::FileFormat file_format_in = boundingmesh::Invalid;
	file_format_in = boundingmesh::getFileFormat(filename_in);
	if(file_format_in == boundingmesh::Off)
		mesh->loadOff(filename_in);
	else if(file_format_in == boundingmesh::Obj)
		mesh->loadObj(filename_in);
	else if(file_format_in == boundingmesh::Wrl)
		mesh->loadWrl(filename_in);
	else if(file_format_in == boundingmesh::Stl)
		mesh->loadStl(filename_in);
	else
	{
		QMessageBox::warning(this, "Bad File Format", "File type cannot be read.", QMessageBox::Ok);
		return false;
	}
	loadView();
	return true;
}

void MainWindow::clickOpen()
{
	std::string filename_in = QFileDialog::getOpenFileName(this, "Open 3D Mesh", QDir::currentPath(), "*.wrl *.obj *.off *.stl").toStdString();
	if(filename_in == "")
		return;
	openFile(filename_in);
}

void MainWindow::clickOpenExample(const std::string &file_name)
{
	mesh.reset(new boundingmesh::Mesh);
	mesh->loadOff(file_name);
	if(mesh->nVertices() == 0)
	{
		QMessageBox::warning(this, "Bad Example File", QString("File %1 cannot be read.").arg(QString::fromStdString(file_name)), QMessageBox::Ok);
		return;
	}
	loadView();
}

void MainWindow::clickOpenExample1()
{
	clickOpenExample("torus.off");
}

void MainWindow::clickOpenExample2()
{
	clickOpenExample("teapot.off");
}

void MainWindow::clickOpenExample3()
{
	clickOpenExample("bunny.off");
}

void MainWindow::clickSave()
{
	std::string filename_out = QFileDialog::getSaveFileName(this, "Save 3D Mesh", QDir::currentPath(), "*.wrl *.obj *.off *.stl").toStdString();
	if(filename_out == "")
		return;
	boundingmesh::FileFormat file_format_out = boundingmesh::Invalid;
	file_format_out = boundingmesh::getFileFormat(filename_out);
	boundingmesh::Mesh copy = *(this->mesh);
	if(file_format_out == boundingmesh::Off)
		copy.writeOff(filename_out);
	else if(file_format_out == boundingmesh::Obj)
		copy.writeObj(filename_out);
	else if(file_format_out == boundingmesh::Wrl)
		copy.writeWrl(filename_out);
	else if(file_format_out == boundingmesh::Stl)
		copy.writeStl(filename_out, false);
	else
	{
		QMessageBox::warning(this, "", "File type cannot be written.", QMessageBox::Ok);
		return;
	}
}

void MainWindow::clickReset()
{
	if(original_mesh != NULL)
	{
		decimator->setMesh(*original_mesh);
		mesh = decimator->getMesh();
		viewer_mesh->replaceByMesh(mesh);
		reload();	
	}
}

void MainWindow::clickOuter()
{
	direction = boundingmesh::Outward;
	if(decimator)
		decimator->setDirection(direction);
}

void MainWindow::clickInner()
{
	direction = boundingmesh::Inward;
	if(decimator)
		decimator->setDirection(direction);
}

void MainWindow::clickAny()
{
	direction = boundingmesh::Any;
	if(decimator)
		decimator->setDirection(direction);
}

void MainWindow::clickClassicQEM()
{
	metric = boundingmesh::ClassicQEM;
	if(decimator)
		decimator->setMetric(metric);
}

void MainWindow::clickModifiedQEM()
{
	metric = boundingmesh::ModifiedQEM;
	if(decimator)
		decimator->setMetric(metric);
}

void MainWindow::clickMinConstant()
{
	metric = boundingmesh::MinimizedConstant;
	if(decimator)
		decimator->setMetric(metric);
}

void MainWindow::clickDiagonalization()
{
	metric = boundingmesh::Diagonalization;
	if(decimator)
		decimator->setMetric(metric);
}


void MainWindow::clickAverage()
{
	metric = boundingmesh::Average;
	if(decimator)
		decimator->setMetric(metric);
}


void MainWindow::clickMidpoint()
{
	initialization = boundingmesh::Midpoint;
	if(decimator)
		decimator->setInitialization(initialization);
}


void MainWindow::clickDistancePrimitives()
{
	initialization = boundingmesh::DistancePrimitives;
	if(decimator)
		decimator->setInitialization(initialization);
}


void MainWindow::clickDecimate()
{
	decimator->doContractions();
	viewer_mesh->replaceByMesh(mesh);
	reload();
}

void MainWindow::clickDecimateMany()
{
	unsigned int n_contractions = 100;
	if(mesh->nVertices() > n_contractions + minimal_vertices)
		decimator->doContractions(n_contractions);
	else if(mesh->nVertices() > minimal_vertices)
	{
		decimator->doContractions(mesh->nVertices() - minimal_vertices);
	}
	viewer_mesh->replaceByMesh(mesh);
	reload();
}

void MainWindow::clickDecimateManyMore()
{
	unsigned int n_contractions = 5000;
	if(mesh->nVertices() > n_contractions + minimal_vertices)
		decimator->doContractions(n_contractions);
	else if(mesh->nVertices() > minimal_vertices)
	{
		decimator->doContractions(mesh->nVertices() - minimal_vertices);
	}
	viewer_mesh->replaceByMesh(mesh);
	reload();
}

void MainWindow::clickDecimateVertices()
{
	bool valid = true;
	if(line_desired_vertices->text() == "")
	{
		line_desired_vertices->setText(line_desired_vertices->placeholderText());
		reload();
	}
	int vertices = line_desired_vertices->text().toInt(&valid);
	if(vertices < 0 || !valid)
	{
		line_desired_error->setText("");
		return;	
	}
	if(!using_error_restriction_)
		decimator->unsetMaximumError();
	unsigned int step = ::std::max(1, (int) mesh->nVertices() / 100);
	while(mesh->nVertices() > vertices + step && decimator->nextError() < maximum_error)
	{
		decimator->setTargetVertices(mesh->nVertices() - step);
		decimator->compute();
		viewer_mesh->replaceByMesh(mesh);
		reload();
	}
	decimator->setTargetVertices(vertices);
	decimator->compute();
	viewer_mesh->replaceByMesh(mesh);
	if(mesh->nVertices() == vertices)
	{
		line_desired_vertices->setText("");
	}
	reload();
}

void MainWindow::clickDecimateError()
{
	if(line_desired_error->text() == "")
	{
		line_desired_error->setText(line_desired_error->placeholderText());
		reload();
	}
	bool valid = true;
	double error = line_desired_error->text().toDouble(&valid);
	if(error < 0 || !valid)
	{
		line_desired_error->setText("");
		return;
	}
	else if(using_error_restriction_ && error > maximum_error)
	{
		queryErrorRestriction();
		if(!using_error_restriction_)
			decimator->setMaximumError(error);
	}
	else
		decimator->setMaximumError(error);
	
	unsigned int step = 1000;
	while(decimator->nextError() < error)
	{
		if(mesh->nVertices() > step)
			decimator->setTargetVertices(mesh->nVertices() - step);
		else
			decimator->setTargetVertices(0);		
		decimator->compute();
		viewer_mesh->replaceByMesh(mesh);
		reload();
	}
	if(using_error_restriction_)
		decimator->setMaximumError(maximum_error);

	line_desired_error->setText("");
	reload();
}

void MainWindow::clickDecimatedSolid()
{
	if(original_mesh != NULL)
	{
		viewer_mesh->setWireframe(false);
		viewer_mesh->replaceByMesh(mesh);
	}
	reload();
}

void MainWindow::clickDecimatedWire()
{
	if(original_mesh != NULL)
	{
		viewer_mesh->setWireframe(true);
		viewer_mesh->replaceByMesh(mesh);
	}
	reload();
}

void MainWindow::clickDecimatedInvis()
{
	if(original_mesh != NULL)
	{
		viewer_mesh->replaceByMesh(::std::make_shared< boundingmesh::Mesh >());
	}
	reload();
}

void MainWindow::clickOriginalSolid()
{
	if(original_mesh != NULL)
	{
		viewer_mesh_original->setWireframe(false);
		viewer_mesh_original->replaceByMesh(original_mesh);
	}
	reload();
}

void MainWindow::clickOriginalWire()
{
	if(original_mesh != NULL)
	{
		viewer_mesh_original->setWireframe(true);
		viewer_mesh_original->replaceByMesh(original_mesh);
	}
	reload();
}

void MainWindow::clickOriginalInvis()
{
	if(original_mesh != NULL)
	{
		viewer_mesh_original->replaceByMesh(::std::make_shared< boundingmesh::Mesh >());
	}
	reload();
}

void
MainWindow::dragEnterEvent(QDragEnterEvent* event)
{
	event->acceptProposedAction();
}

void
MainWindow::dropEvent(QDropEvent* event)
{
	if (event->mimeData()->hasUrls())
	{
		if (event->mimeData()->urls().size() > 0)
		{
			QString filename = event->mimeData()->urls()[0].toLocalFile();
			if(openFile(filename.toStdString()))
				event->acceptProposedAction();
		}
	}
}

#include <QtOpenGL/QGLWidget>
#include <Inventor/nodes/SoEnvironment.h>

MainWindow::MainWindow(QWidget* parent, QApplication* app) :
	app(app),
	viewer(NULL),
	mesh(NULL),
	original_mesh(NULL),
	decimator(NULL),
	direction(boundingmesh::Outward),
	metric(boundingmesh::Average),
	initialization(boundingmesh::Midpoint),
	using_error_restriction_(true)
{
	SoQt::init(this);
	SoDB::init();
	
	setWindowIcon(QPixmap(":/icon.png"));
	setCentralWidget(&viewer_widget);
	
	QDockWidget *dock_view = new QDockWidget();
	QDockWidget *dock_decimate = new QDockWidget();

	QVBoxLayout *dock_layout_view = new QVBoxLayout;
	QVBoxLayout *dock_layout_decimate = new QVBoxLayout;
	QVBoxLayout *group_file_layout = new QVBoxLayout;
	QVBoxLayout *group_reset_layout = new QVBoxLayout;
	QVBoxLayout *group_options_layout = new QVBoxLayout;
	QVBoxLayout *group_decimate_layout = new QVBoxLayout;
	QVBoxLayout *group_view_layout = new QVBoxLayout;
	QGridLayout *group_results_layout = new QGridLayout();
	QGroupBox *group_file = new QGroupBox("File");
	QGroupBox *group_reset = new QGroupBox("Reset");
	QGroupBox *group_options = new QGroupBox("Options");
	QGroupBox *group_decimate = new QGroupBox("Generate");
	QGroupBox *group_view = new QGroupBox("View");
	QGroupBox *group_results = new QGroupBox("Results");
	QWidget *dock_widget_view = new QWidget;
	QWidget *dock_widget_decimate = new QWidget;

	QVBoxLayout *group_options_restrictions_layout = new QVBoxLayout;
	QGroupBox *group_options_restrictions = new QGroupBox("Restriction");
	radio_outer = new QRadioButton("Outer Bounding Mesh");
	radio_any = new QRadioButton("Unrestricted Simplification");
	radio_inner = new QRadioButton("Inner Bounding Mesh");
	QVBoxLayout *group_options_metric_layout = new QVBoxLayout;
	QGroupBox *group_options_metric = new QGroupBox("Metric");
	radio_classicQEM = new QRadioButton("Classic QEM");
	radio_modifiedQEM = new QRadioButton("Modified QEM");
	radio_MinConstant = new QRadioButton("MinimizedConstant");
	radio_Diagonalization = new QRadioButton("Diagonalization");
	radio_Average = new QRadioButton("Average");

	QVBoxLayout *group_options_init_layout = new QVBoxLayout;
	QGroupBox *group_options_init = new QGroupBox("Initialization");
	radio_Midpoint = new QRadioButton("Midpoint");
	radio_DistancePrimitives = new QRadioButton("DistancePrimitives");
	
	QPushButton *button_open = new QPushButton("&Open File");
	QPushButton *button_open_example1 = new QPushButton("Open Example \"&Torus\"");
	QPushButton *button_open_example2 = new QPushButton("Open Example \"Teapot\"");
	QPushButton *button_open_example3 = new QPushButton("Open Example \"&Bunny\"");
	QPushButton *button_save = new QPushButton("&Save As...");
	
	button_reset = new QPushButton("&Reset");
	button_reset->setDisabled(true);

	button_decimate = new QPushButton("&Decimate Edge");
	button_decimate->setDisabled(true);
	button_decimate_many = new QPushButton("D&ecimate 100 Edges");
	button_decimate_many->setDisabled(true);
	button_decimate_many_more = new QPushButton("De&cimate 5000 Edges");
	button_decimate_many_more->setDisabled(true);

	QHBoxLayout *group_view_decimated_layout = new QHBoxLayout;
	QHBoxLayout *group_view_original_layout = new QHBoxLayout;
	QGroupBox *group_view_decimated = new QGroupBox("Decimated Mesh");
	QGroupBox *group_view_original = new QGroupBox("Original");

	radio_decimated_solid = new QRadioButton("Solid");
	radio_decimated_wire = new QRadioButton("Wireframe");
	radio_decimated_invis = new QRadioButton("Invisible");
	radio_original_solid = new QRadioButton("Solid");
	radio_original_wire = new QRadioButton("Wireframe");
	radio_original_invis = new QRadioButton("Invisible");

	QLabel *label_result_vertices = new QLabel("Vertices");
	QLabel *label_result_error = new QLabel("Error");
	line_result_vertices = new QLineEdit;
	line_result_error = new QLineEdit;
	QSpacerItem *spacer_item = new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Expanding);
	QSpacerItem *spacer_item2 = new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Expanding);

	QVBoxLayout *group_decimate_manual_layout = new QVBoxLayout;
	QHBoxLayout *group_decimate_vertices_layout = new QHBoxLayout;
	QHBoxLayout *group_decimate_error_layout = new QHBoxLayout;
	QGroupBox *group_decimate_manual = new QGroupBox("Manually");
	QGroupBox *group_decimate_vertices = new QGroupBox("Number of Vertices");
	QGroupBox *group_decimate_error = new QGroupBox("Mesh Error");
	line_desired_vertices = new CustomLineEdit;
	line_desired_error = new CustomLineEdit;
	button_decimate_vertices = new QPushButton("Decimate");
	button_decimate_vertices->setDisabled(true);
	button_decimate_error = new QPushButton("Decimate");
	button_decimate_error->setDisabled(true);

	this->addDockWidget(Qt::LeftDockWidgetArea, dock_view);
	dock_view->setWidget(dock_widget_view);
	dock_widget_view->setLayout(dock_layout_view);
	dock_layout_view->addWidget(group_file);
	group_file->setLayout(group_file_layout);
	group_file_layout->addWidget(button_open);
	group_file_layout->addWidget(button_open_example1);
	group_file_layout->addWidget(button_open_example2);
	group_file_layout->addWidget(button_open_example3);
	group_file_layout->addWidget(button_save);

	dock_layout_view->addWidget(group_reset);
	group_reset->setLayout(group_reset_layout);
	group_reset_layout->addWidget(button_reset);

	dock_layout_view->addWidget(group_results);
	group_results->setLayout(group_results_layout);
	group_results_layout->addWidget(label_result_vertices, 0, 0);
	group_results_layout->addWidget(line_result_vertices, 0, 1);
	line_result_vertices->setReadOnly(true);
	group_results_layout->addWidget(label_result_error, 1, 0);
	group_results_layout->addWidget(line_result_error, 1, 1);
	line_result_error->setReadOnly(true);

	
	dock_layout_view->addWidget(group_view);
	group_view->setLayout(group_view_layout);
	group_view_layout->addWidget(group_view_decimated);
	group_view_decimated->setLayout(group_view_decimated_layout);
	group_view_decimated_layout->addWidget(radio_decimated_solid);
	group_view_decimated_layout->addWidget(radio_decimated_wire);
	group_view_decimated_layout->addWidget(radio_decimated_invis);
	radio_decimated_wire->setChecked(true);

	group_view_layout->addWidget(group_view_original);
	group_view_original->setLayout(group_view_original_layout);
	group_view_original_layout->addWidget(radio_original_solid);
	group_view_original_layout->addWidget(radio_original_wire);
	group_view_original_layout->addWidget(radio_original_invis);
	radio_original_solid->setChecked(true);

	dock_layout_view->addItem(spacer_item);


	this->addDockWidget(Qt::RightDockWidgetArea, dock_decimate);
	dock_decimate->setWidget(dock_widget_decimate);
	dock_widget_decimate->setLayout(dock_layout_decimate);

	dock_layout_decimate->addWidget(group_options);
	group_options->setLayout(group_options_layout);
	group_options_layout->addWidget(group_options_restrictions);
	group_options_restrictions->setLayout(group_options_restrictions_layout);
	group_options_restrictions_layout->addWidget(radio_outer);
	radio_outer->setChecked(true);
	group_options_restrictions_layout->addWidget(radio_any);
	group_options_restrictions_layout->addWidget(radio_inner);

	group_options_layout->addWidget(group_options_metric);
	group_options_metric->setLayout(group_options_metric_layout);
	group_options_metric_layout->addWidget(radio_classicQEM);
	group_options_metric_layout->addWidget(radio_modifiedQEM);
	group_options_metric_layout->addWidget(radio_MinConstant);
	group_options_metric_layout->addWidget(radio_Diagonalization);
	group_options_metric_layout->addWidget(radio_Average);
	radio_Average->setChecked(true);

	group_options_layout->addWidget(group_options_init);
	group_options_init->setLayout(group_options_init_layout);
	group_options_init_layout->addWidget(radio_Midpoint);
	radio_Midpoint->setChecked(true);
	group_options_init_layout->addWidget(radio_DistancePrimitives);

	group_options_metric_layout->addWidget(radio_Diagonalization);
	group_options_metric_layout->addWidget(radio_Average);

	dock_layout_decimate->addWidget(group_decimate);
	group_decimate->setLayout(group_decimate_layout);

	group_decimate_layout->addWidget(group_decimate_manual);
	group_decimate_manual->setFlat(true);
	group_decimate_manual->setLayout(group_decimate_manual_layout);
	group_decimate_manual_layout->addWidget(button_decimate);
	group_decimate_manual_layout->addWidget(button_decimate_many);
	group_decimate_manual_layout->addWidget(button_decimate_many_more);

	group_decimate_layout->addWidget(group_decimate_vertices);
	group_decimate_vertices->setFlat(true);
	group_decimate_vertices_layout->addWidget(line_desired_vertices);
	group_decimate_vertices_layout->addWidget(button_decimate_vertices);
	group_decimate_vertices->setLayout(group_decimate_vertices_layout);

	group_decimate_layout->addWidget(group_decimate_error);
	group_decimate_error->setFlat(true);
	group_decimate_error_layout->addWidget(line_desired_error);
	group_decimate_error_layout->addWidget(button_decimate_error);
	group_decimate_error->setLayout(group_decimate_error_layout);

	dock_layout_decimate->addItem(spacer_item2);

	QGLFormat format;
	format.setAlpha(true);
	format.setSampleBuffers(true);
	format.setSamples(4);
	QGLFormat::setDefaultFormat(format);
	
	
	
	scene_graph = new SoGroup;
	scene_graph->ref();

	SoEnvironment* environment = new SoEnvironment;
	environment->ambientIntensity = 1.0;
	environment->ambientColor.setValue(1.0, 1.0, 1.0);
 	scene_graph->addChild(environment);
	
	SoGroup *group_decimated = new SoGroup;
	SoMaterial *material_decimated = new SoMaterial;
	material_decimated->diffuseColor.setValue(0.0, 0.0, 0.0);
	group_decimated->addChild(material_decimated);
	viewer_mesh.reset(new ViewerMesh);
	viewer_mesh->setWireframe(true);
	group_decimated->addChild(viewer_mesh->model_root);
	scene_graph->addChild(group_decimated);

	SoGroup *group_original = new SoGroup;
	SoMaterial *material_original = new SoMaterial;
	material_original->diffuseColor.setValue(0.0, 0.0, 1.0);
	group_original->addChild(material_original);
	SoPolygonOffset *offset_original = new SoPolygonOffset;
	offset_original->factor = 0;
	offset_original->units = 10;
	group_original->addChild(offset_original);
	viewer_mesh_original.reset(new ViewerMesh);
	viewer_mesh_original->setWireframe(false);
	group_original->addChild(viewer_mesh_original->model_root);
	scene_graph->addChild(group_original);


	viewer = new SoQtExaminerViewer(&viewer_widget);
	//viewer->setDecoration(true);
	viewer->setBackgroundColor(SbColor(255, 255, 255));
	viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	viewer->setDrawStyle(SoQtViewer::STILL, SoQtViewer::VIEW_AS_IS);
	viewer->setDrawStyle(SoQtViewer::INTERACTIVE, SoQtViewer::VIEW_SAME_AS_STILL);
	viewer->setSceneGraph(scene_graph);
	viewer->setHeadlight(TRUE);
	setFocusProxy(this->viewer->getWidget());
	
	resize(1120, 630);
	setWindowTitle("Bounding Mesh GUI " BOUNDINGMESH_VERSION);
	setAcceptDrops(true);
	
	connect(button_open, SIGNAL(clicked()), this, SLOT(clickOpen()));
	connect(button_open_example1, SIGNAL(clicked()), this, SLOT(clickOpenExample1()));
	connect(button_open_example2, SIGNAL(clicked()), this, SLOT(clickOpenExample2()));
	connect(button_open_example3, SIGNAL(clicked()), this, SLOT(clickOpenExample3()));
	connect(button_save, SIGNAL(clicked()), this, SLOT(clickSave()));
	connect(button_reset, SIGNAL(clicked()), this, SLOT(clickReset()));
	connect(radio_outer, SIGNAL(clicked()), this, SLOT(clickOuter()));
	connect(radio_inner, SIGNAL(clicked()), this, SLOT(clickInner()));
	connect(radio_any, SIGNAL(clicked()), this, SLOT(clickAny()));
	connect(radio_classicQEM, SIGNAL(clicked()), this, SLOT(clickClassicQEM()));
	connect(radio_modifiedQEM, SIGNAL(clicked()), this, SLOT(clickModifiedQEM()));
	connect(radio_MinConstant, SIGNAL(clicked()), this, SLOT(clickMinConstant()));
	connect(radio_Diagonalization, SIGNAL(clicked()), this, SLOT(clickDiagonalization()));
	connect(radio_Average, SIGNAL(clicked()), this, SLOT(clickAverage()));
	connect(radio_Midpoint, SIGNAL(clicked()), this, SLOT(clickMidpoint()));
	connect(radio_DistancePrimitives, SIGNAL(clicked()), this, SLOT(clickDistancePrimitives()));
	connect(button_decimate, SIGNAL(clicked()), this, SLOT(clickDecimate()));
	connect(button_decimate_many, SIGNAL(clicked()), this, SLOT(clickDecimateMany()));
	connect(button_decimate_many_more, SIGNAL(clicked()), this, SLOT(clickDecimateManyMore()));
	connect(button_decimate_vertices, SIGNAL(clicked()), this, SLOT(clickDecimateVertices()));
	connect(button_decimate_error, SIGNAL(clicked()), this, SLOT(clickDecimateError()));
	connect(radio_decimated_solid, SIGNAL(clicked()), this, SLOT(clickDecimatedSolid()));
	connect(radio_decimated_wire, SIGNAL(clicked()), this, SLOT(clickDecimatedWire()));
	connect(radio_decimated_invis, SIGNAL(clicked()), this, SLOT(clickDecimatedInvis()));
	connect(radio_original_solid, SIGNAL(clicked()), this, SLOT(clickOriginalSolid()));
	connect(radio_original_wire, SIGNAL(clicked()), this, SLOT(clickOriginalWire()));
	connect(radio_original_invis, SIGNAL(clicked()), this, SLOT(clickOriginalInvis()));
}

MainWindow::~MainWindow()
{
	scene_graph->unref();
}

void MainWindow::reload()
{
	if(original_mesh != NULL)
		button_reset->setDisabled(false);

	if(mesh->nVertices() < minimal_vertices)
		button_decimate_many->setEnabled(false);
	else 
		button_decimate_many->setEnabled(true);

	if(mesh->nVertices() < minimal_vertices + 3 * number_decimate_many)
		button_decimate_many_more->setEnabled(false);
	else
		button_decimate_many_more->setEnabled(true);

	if(using_error_restriction_ && decimator->nextError() > maximum_error)
	{
		queryErrorRestriction();
	}

	unsigned int recommended_vertices = mesh->nVertices() / 2;
	boundingmesh::Real recommended_error = 0;
	if(decimator->currentError() < 0){
 		recommended_error = maximum_error / 100;	
		line_result_error->setText(QString("No error"));
	}
	else{
		line_result_error->setText(QString::number(decimator->currentError()));
		if(using_error_restriction_){
			recommended_error = (1 * maximum_error+ 81 * decimator->currentError()) / 100 + 2 * 9 *std::sqrt(maximum_error * decimator->currentError()) / 100;
			if(recommended_error < decimator->nextError())
				recommended_error = recommended_error * 1.1 * 1.1;
		}
		else{
			recommended_error = decimator->currentError() * 1.1 * 1.1;
		}
	}
	line_desired_vertices->setPlaceholderText(QString::number(recommended_vertices));
	line_desired_error->setPlaceholderText(QString::number(recommended_error));
	line_result_vertices->setText(QString::number(mesh->nVertices()));
	viewer->render();
	app->processEvents();
}
void MainWindow::queryErrorRestriction()
{
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, "High Mesh Error", "You are leaving the recommended error-limits. Unlock mesh error?",
			                QMessageBox::Yes|QMessageBox::No);
	if (reply == QMessageBox::Yes) 
	{
		decimator->unsetMaximumError();
		using_error_restriction_ = false;
	}
}

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	QObject::connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
	MainWindow window(NULL, &app);
	window.show();
	
	return app.exec();
}

