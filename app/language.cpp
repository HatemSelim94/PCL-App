#include "language.h"

#include <QCoreApplication>
#include <QLibraryInfo>
#include <QLocale>
#include <QSettings>

#include <memory>

namespace pclapp::app {

namespace {

constexpr const char* kSettingsKey = "ui/language";

// The translators currently installed. Replaced wholesale on every change;
// removing the old ones first is what makes a switch back to English work.
std::unique_ptr<QTranslator>& appTranslator() {
    static std::unique_ptr<QTranslator> translator;
    return translator;
}
std::unique_ptr<QTranslator>& qtTranslator() {
    static std::unique_ptr<QTranslator> translator;
    return translator;
}

QString translationsDirectory() {
    return QCoreApplication::applicationDirPath() + "/translations";
}

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
QString qtTranslationsDirectory() {
    return QLibraryInfo::path(QLibraryInfo::TranslationsPath);
}
#else
QString qtTranslationsDirectory() {
    return QLibraryInfo::location(QLibraryInfo::TranslationsPath);
}
#endif

}  // namespace

const std::vector<Language>& availableLanguages() {
    static const std::vector<Language> languages = {
        {"en", QStringLiteral("English")},
        {"de", QStringLiteral("Deutsch")},
        {"fr", QStringLiteral("Français")},
        {"es", QStringLiteral("Español")},
    };
    return languages;
}

QString savedLanguageCode() {
    return QSettings().value(kSettingsKey).toString();
}

void saveLanguageCode(const QString& code) {
    QSettings settings;
    if (code.isEmpty()) {
        settings.remove(kSettingsKey);
    } else {
        settings.setValue(kSettingsKey, code);
    }
}

void applyLanguage(const QString& code) {
    const QLocale locale = code.isEmpty() ? QLocale::system() : QLocale(code);

    // Removing a translator also triggers LanguageChange, so the widgets fall
    // back to their English source strings when no translation gets loaded.
    for (auto* holder : {&appTranslator(), &qtTranslator()}) {
        if (*holder) {
            QCoreApplication::removeTranslator(holder->get());
            holder->reset();
        }
    }

    // Qt's own strings (file dialog, message box buttons) first, then ours.
    auto qt = std::make_unique<QTranslator>();
    if (qt->load(locale, "qtbase", "_", qtTranslationsDirectory())) {
        QCoreApplication::installTranslator(qt.get());
        qtTranslator() = std::move(qt);
    }
    auto app = std::make_unique<QTranslator>();
    if (app->load(locale, "pclapp", "_", translationsDirectory())) {
        QCoreApplication::installTranslator(app.get());
        appTranslator() = std::move(app);
    }
}

}  // namespace pclapp::app
