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

#include <QAction>
#include <QMainWindow>
#include <QMenuBar>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QPushButton>
#include <QDockWidget>
#include <QSpacerItem>
#include <QLabel>
#include <QString>
#include <QIcon>
#include <QGroupBox>
#include <QCheckBox>

#include <memory>
#include <string>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>

#include <boundingmesh.h>

#include "ViewerMesh.h"

class CustomLineEdit : public QLineEdit
{
	Q_OBJECT
public:
	CustomLineEdit(QWidget * parent = 0);
	
private:
	void focusInEvent(QFocusEvent *e);
	void mousePressEvent(QMouseEvent *me);
    	bool select_on_mouse_press_;
};

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	MainWindow(QWidget* parent = NULL, QApplication* app = NULL);
	virtual ~MainWindow();
	
	void loadView();
	bool openFile(const std::string &file_name);
	
	::std::shared_ptr< boundingmesh::Mesh > mesh;
	::std::shared_ptr< boundingmesh::Mesh > original_mesh;
	::std::shared_ptr< boundingmesh::Decimator > decimator;
	boundingmesh::DecimationDirection direction;
	boundingmesh::Metric metric;
	boundingmesh::Initialization initialization;
	
	SoQtExaminerViewer *viewer;
	SoGroup *scene_graph;
	::std::shared_ptr< ViewerMesh > viewer_mesh;
	::std::shared_ptr< ViewerMesh > viewer_mesh_original;
	QWidget central_widget;
	QWidget viewer_widget;

public slots:
	void reload();

	void clickOpen();
	void clickOpenExample(const std::string &file_name);
	void clickOpenExample1();
	void clickOpenExample2();
	void clickOpenExample3();
	void clickSave();
	void clickReset();
	void clickOuter();
	void clickInner();
	void clickAny();
	void clickClassicQEM();
	void clickModifiedQEM();
	void clickMinConstant();
	void clickDiagonalization();
	void clickAverage();
	void clickMidpoint();
	void clickDistancePrimitives();
	void clickDecimate();
	void clickDecimateMany();
	void clickDecimateManyMore();
	void clickDecimateVertices();
	void clickDecimateError();
	void clickDecimatedSolid();
	void clickDecimatedWire();
	void clickDecimatedInvis();
	void clickOriginalSolid();
	void clickOriginalWire();
	void clickOriginalInvis();
	
protected:
	void dragEnterEvent(QDragEnterEvent* event);
	void dropEvent(QDropEvent* event);
	
private:
	QApplication* app;
	
	QRadioButton *radio_outer;
	QRadioButton *radio_any;
	QRadioButton *radio_inner;
	QRadioButton *radio_classicQEM;
	QRadioButton *radio_modifiedQEM;
	QRadioButton *radio_MinConstant;
	QRadioButton *radio_Diagonalization;
	QRadioButton *radio_Average;
	QRadioButton *radio_Midpoint;
	QRadioButton *radio_DistancePrimitives;
	QRadioButton *radio_decimated_solid;
	QRadioButton *radio_decimated_wire;
	QRadioButton *radio_decimated_invis;
	QRadioButton *radio_original_solid;
	QRadioButton *radio_original_wire;
	QRadioButton *radio_original_invis;
	QLineEdit *line_result_vertices;
	QLineEdit *line_result_error;
	CustomLineEdit *line_desired_vertices;
	CustomLineEdit *line_desired_error;
	QPushButton *button_reset;
	QPushButton *button_decimate;
	QPushButton *button_decimate_many;
	QPushButton *button_decimate_many_more;
	QPushButton *button_decimate_vertices;
	QPushButton *button_decimate_error;

	bool using_error_restriction_;
	void queryErrorRestriction();
};


