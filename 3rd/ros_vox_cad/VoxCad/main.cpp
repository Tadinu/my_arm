/*******************************************************************************
Copyright (c) 2010, Jonathan Hiller
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name if its contributors may not be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <QtWidgets/QApplication>
#include "VoxCad.h"

#ifdef MY_ARM_GAZEBO_TRANSPORT
#include "GazeboArmCommander.h"
#include "GazeboImageListener.h"
#endif

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
#ifdef MY_ARM_GAZEBO_TRANSPORT
    QMutex mutex;
    gazebo::common::Image image;
    VoxCad w(nullptr, mutex, image);
#else
    VoxCad w;
#endif
	if (argc > 1){
		QString Path = argv[1];
		if (Path.right(3) == "vxc") w.MainObj.LoadVXCFile(Path.toStdString()); //open a file if we've passed the args...
		if (Path.right(3) == "vxp") w.MainObj.LoadVXPFile(Path.toStdString()); //open a file if we've passed the args...
		else if (Path.right(3) == "vxa") w.MainSim.LoadVXAFile(Path.toStdString()); //open a file if we've passed the args...

	}

#ifdef MY_ARM_GAZEBO_TRANSPORT
#if 1
    GazeboMyArmCommander commander;
    ScenePtr scene = commander.createScene(CGZ_RENDERING_ENGINE);
    commander.addSceneContent(scene);

    // CAMERA --
    CameraPtr camera = commander.createCamera(scene);
    VisualPtr root   = scene->GetRootVisual();
    root->AddChild(camera);

    std::vector<CameraPtr> cameras;
    cameras.push_back(camera);

    w.runGazeboCamera(cameras);
#else
    GazeboImageListener imageListener(argc, argv, mutex, image);
    QObject::connect(&imageListener, SIGNAL(newImageArrived()), &w, SLOT(onGazeboImageArrived()));
    imageListener.start();
#endif
#endif

	w.show();

	return a.exec();
}
