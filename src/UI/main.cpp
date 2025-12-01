#include "../Base/config.h"
#include "../Base/logging.h"
#include "MainWindow.h"
#include <QApplication>          
#include <QTranslator>          

int main(int argc, char* argv[])
{
	Config::getInstance("../settings.ini");
	InitializeGlog(argv);

	QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

	QApplication App(argc, argv);
	QTranslator translator;
	translator.load("C:\\Users\\WTG\\code\\OntheFlySfM\\src\\UI\\Translation_zh_CN.qm");
	App.installTranslator(&translator);

	MainWindow w;
	w.show();

	return App.exec();
}