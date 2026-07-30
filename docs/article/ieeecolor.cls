%%
%% ieeecolor.cls — IEEEtran wrapper with color support
%% Provides the same interface as IEEEtran but with color definitions
%%
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{ieeecolor}[2024/01/01 IEEEtran wrapper with color support]

%% Pass all options to IEEEtran
\DeclareOption*{%
  \PassOptionsToClass{\CurrentOption}{IEEEtran}%
}
\ProcessOptions\relax

%% Load IEEEtran
\LoadClass[journal,twoside]{IEEEtran}[2023/06/01]

%% Define IEEE standard colors
\RequirePackage{xcolor}

%% IEEE standard colors (from IEEEtran color configuration)
\definecolor{IEEEBlue}{cmyk}{1,.60,0,.40}
\definecolor{IEEEPurple}{cmyk}{0.45,0.86,0,0.04}
\definecolor{IEEEOrange}{cmyk}{0.03,0.63,0.95,0}
\definecolor{IEEEGreen}{cmyk}{1,0,1,.23}
\definecolor{IEEERed}{cmyk}{0,1,1,0}
\definecolor{IEEEDarkBlue}{cmyk}{1,0.62,0.10,0.52}
\definecolor{IEEELightBlue}{cmyk}{0.14,0.26,0,0.05}
\definecolor{IEEEYellow}{cmyk}{0,0.16,0.89,0}
\definecolor{IEEEMagenta}{cmyk}{0,0.46,0,0.22}

%% Abstract background color (used in spa_rf_vio_cn.tex)
\definecolor{abstractbg}{rgb}{0.89804,0.94510,0.83137}

\endinput
