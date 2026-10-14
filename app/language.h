#pragma once
// User-interface language: which translations exist, which one is chosen, and
// installing the matching QTranslator. The choice is stored with QSettings.
//
// Translation files are looked up in <application dir>/translations, where
// the build copies them (see app/CMakeLists in the top-level CMakeLists.txt).

#include <QString>
#include <QTranslator>

#include <vector>

namespace pclapp::app {

struct Language {
    QString code;  ///< BCP-47 style code used in file names, e.g. "de"; empty = system default
    QString nativeName;  ///< Shown in the menu, never translated: "Deutsch", not "German"
};

/// The languages the application ships, English first. Does not include the
/// "system default" entry, which the menu adds itself.
const std::vector<Language>& availableLanguages();

/// The code saved by the user, or empty for "follow the system".
QString savedLanguageCode();
void saveLanguageCode(const QString& code);

/// Installs the application and Qt translations for `code` (empty: the
/// system locale) on the running application, replacing the previous ones.
/// Qt then sends a LanguageChange event to every widget.
void applyLanguage(const QString& code);

}  // namespace pclapp::app
