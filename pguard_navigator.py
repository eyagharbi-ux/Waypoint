"""
==============================================================================
 pguard_navigator.py  -  Interface graphique de navigation PGuard
==============================================================================
 Auteur  : Stagiaire Enova Robotics
 Version : 2.0.0
 Projet  : Robot de securite PGuard - Navigation autonome

 Description :
     Interface graphique interactive permettant de :
       - Visualiser la carte OpenStreetMap chargee depuis map.osm
       - Poser un point de DEPART et un point d'ARRIVEE par clic
       - Generer et afficher la trajectoire (via route_engine.py)
       - Annuler les actions etape par etape (bouton Retour)
       - Exporter la trajectoire en JSON

     Architecture :
       - PyQt6 fournit la fenetre principale, les boutons et la barre d'etat.
       - Un QWebEngineView affiche une carte Folium (HTML/Leaflet.js).
       - Un mini serveur HTTP (http.server) tourne dans un thread daemon
         et sert la carte via http://127.0.0.1:PORT/ pour contourner les
         restrictions de securite de QtWebEngine sur les fichiers locaux.
       - La communication JS -> Python se fait via QWebChannel.

 IMPORTANT - Ordre d'initialisation Qt (critique pour Windows) :
       1. Importer QtWebEngineWidgets AVANT QApplication
       2. Appeler AA_ShareOpenGLContexts AVANT QApplication
       3. Creer QApplication
       4. Creer la fenetre principale

 Dependances :
     PyQt6, PyQt6-WebEngine, folium, route_engine  (voir requirements.txt)
==============================================================================
"""

# ---------------------------------------------------------------------------
# ETAPE 1 : initialisation QtWebEngine AVANT tout le reste (critique Windows)
# ---------------------------------------------------------------------------
import sys

# L'import de QtWebEngineWidgets DOIT preceder la creation de QApplication
# sur Windows, sinon le rendu reste blanc (bug connu PyQt6-WebEngine).
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)

# ---------------------------------------------------------------------------
# Imports standard
# ---------------------------------------------------------------------------
import json
import socket
import threading
import http.server
from pathlib import Path
from datetime import datetime

# --- NetworkX (pour le type hint du graphe routier) ---
import networkx as nx

# --- PyQt6 ---
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QStatusBar, QFileDialog, QMessageBox,
    QFrame, QDialog, QSizePolicy, QComboBox, QInputDialog, QScrollArea,
)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebChannel import QWebChannel
from PyQt6.QtCore import (
    QObject, pyqtSlot, QUrl, pyqtSignal
)
from PyQt6.QtGui import QFont

# --- Module Back-end ---
from route_engine import PGuardRouteEngine, Waypoint, RouteResult, calculate_path_distance

# --- Folium pour la generation de la carte interactive ---
import folium


# ===========================================================================
# Constantes de configuration de l'interface
# ===========================================================================

def _application_dir() -> Path:
    """Repertoire racine (sources ou bundle PyInstaller)."""
    if getattr(sys, "frozen", False):
        return Path(sys._MEIPASS)
    return Path(__file__).resolve().parent


# Chemin par defaut du fichier OSM (meme repertoire que ce script / bundle)
DEFAULT_OSM_FILE = _application_dir() / "map.osm"

# Fichier HTML de la carte (genere dans le repertoire du projet)
MAP_HTML_FILE = _application_dir() / "_map_cache.html"

# --- Palette de couleurs PGuard (theme sombre professionnel) ---
COLOR_BG_DARK        = "#0D1117"
COLOR_BG_PANEL       = "#161B22"
COLOR_BG_CARD        = "#21262D"
COLOR_ACCENT         = "#238636"
COLOR_ACCENT_HOVER   = "#2EA043"
COLOR_DANGER         = "#DA3633"
COLOR_INFO           = "#1F6FEB"
COLOR_TEXT_PRIMARY   = "#F0F6FC"
COLOR_TEXT_SECONDARY = "#8B949E"
COLOR_BORDER         = "#30363D"
COLOR_START_MARKER   = "#00C851"
COLOR_END_MARKER     = "#FF4444"
COLOR_ROUTE_MAIN     = "#1F6FEB"
COLOR_ROUTE_ALT      = "#6E7681"
COLOR_NAV_NETWORK    = "#FF9F1C"


# ===========================================================================
# Mini serveur HTTP local (contourne les restrictions file:// de WebEngine)
# ===========================================================================

def _find_free_port() -> int:
    """Trouve un port TCP libre sur la machine locale."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def start_local_http_server(directory: str) -> int:
    """
    Demarre un serveur HTTP simple dans un thread daemon qui sert
    les fichiers du repertoire specifie.

    Ce serveur permet a QtWebEngine de charger la carte Folium via
    http://127.0.0.1:PORT/ au lieu de file://, evitant ainsi les
    restrictions CORS et de securite de Chromium embarque.

    Args:
        directory : Repertoire racine a servir.

    Returns:
        port (int) : Le port TCP sur lequel le serveur ecoute.
    """
    port = _find_free_port()

    # Classe handler definie via closure pour capturer 'directory'
    # (on ne peut pas heriter de functools.partial — ce n'est pas une classe)
    class SilentHandler(http.server.SimpleHTTPRequestHandler):
        """Handler HTTP qui sert 'directory' et supprime les logs console."""

        def __init__(self, *args, **kwargs):
            # Injecter le repertoire cible via le mot-cle 'directory'
            super().__init__(*args, directory=directory, **kwargs)

        def log_message(self, fmt, *args):
            pass  # Silence : ne pas polluer le terminal avec les requetes HTTP

    server = http.server.HTTPServer(("127.0.0.1", port), SilentHandler)

    # Thread daemon : s'arrete automatiquement a la fermeture de l'app
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    print(f"[HTTP Server] Serveur local : http://127.0.0.1:{port}/")
    return port


# ===========================================================================
# Generateur de carte Folium
# ===========================================================================

def build_folium_map(
    center_lat: float,
    center_lon: float,
    zoom: int = 16,
    start_wp: "Waypoint | None" = None,
    end_wp:   "Waypoint | None" = None,
    via_wps:  "list[Waypoint] | None" = None,
    route_result: "RouteResult | None" = None,
    selected_route_idx: int = 0,
    graph: "nx.Graph | None" = None,
) -> None:
    """
    Genere une carte Folium et la sauvegarde dans MAP_HTML_FILE.

    La carte inclut :
      - Fond de carte OpenStreetMap.
      - Marqueurs colores pour le depart (vert) et l'arrivee (rouge).
      - Marqueurs numerotes pour les points intermediaires imposes.
      - Polylignes pour la route active (bleue) et les autres (grises).
      - Code JavaScript pour la communication QWebChannel (clics -> Python).

    Args:
        center_lat   : Latitude du centre.
        center_lon   : Longitude du centre.
        zoom         : Niveau de zoom initial.
        start_wp     : Waypoint de depart (ou None).
        end_wp       : Waypoint d'arrivee (ou None).
        via_wps      : Points intermediaires imposes par l'utilisateur (ou None).
        route_result : Resultat de l'itineraire (ou None).
        selected_route_idx : Index de la route selectionnee (0=principale, 1+=alternative).
        graph        : Le graphe des voies autorisees.
    """
    # --- Creation de la carte Folium ---
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=zoom,
        tiles="OpenStreetMap",
        prefer_canvas=True,
    )

    # --- Dessiner le reseau routier navigable autorise (en arriere-plan) ---
    if graph:
        segments = []
        for u, v in graph.edges:
            lat_u = graph.nodes[u].get("lat")
            lon_u = graph.nodes[u].get("lon")
            lat_v = graph.nodes[v].get("lat")
            lon_v = graph.nodes[v].get("lon")
            if None not in (lat_u, lon_u, lat_v, lon_v):
                segments.append([[lat_u, lon_u], [lat_v, lon_v]])
        if segments:
            folium.PolyLine(
                locations=segments,
                color=COLOR_NAV_NETWORK,
                weight=3.5,
                opacity=0.45,
                tooltip="Voie navigable autorisee",
            ).add_to(m)

    # --- Marqueur de depart ---
    if start_wp:
        folium.Marker(
            location=[start_wp.lat, start_wp.lon],
            popup=folium.Popup(
                f"<b>Point de DEPART</b><br>"
                f"Lat: {start_wp.lat:.6f}<br>Lon: {start_wp.lon:.6f}",
                max_width=200,
            ),
            tooltip="Point de depart",
            icon=folium.Icon(color="green", icon="play", prefix="fa"),
        ).add_to(m)

    # --- Marqueurs des points intermediaires imposes ---
    if via_wps:
        for idx, via_wp in enumerate(via_wps, start=1):
            folium.Marker(
                location=[via_wp.lat, via_wp.lon],
                popup=folium.Popup(
                    f"<b>Point intermediaire {idx}</b><br>"
                    f"Lat: {via_wp.lat:.6f}<br>Lon: {via_wp.lon:.6f}",
                    max_width=200,
                ),
                tooltip=f"Point intermediaire {idx}",
                icon=folium.Icon(color="orange", icon="circle", prefix="fa"),
            ).add_to(m)

    # --- Marqueur d'arrivee ---
    if end_wp:
        folium.Marker(
            location=[end_wp.lat, end_wp.lon],
            popup=folium.Popup(
                f"<b>Point d'ARRIVEE</b><br>"
                f"Lat: {end_wp.lat:.6f}<br>Lon: {end_wp.lon:.6f}",
                max_width=200,
            ),
            tooltip="Point d'arrivee",
            icon=folium.Icon(color="red", icon="flag", prefix="fa"),
        ).add_to(m)

    # --- Dessiner les routes si route_result existe ---
    if route_result:
        # Preparer la liste de toutes les routes
        routes = []
        # Route principale (index 0)
        routes.append((route_result.waypoints, 0, "Itineraire principal"))
        # Alternatives (index 1, 2, ...)
        for idx, alt_wps in enumerate(route_result.alternatives):
            routes.append((alt_wps, idx + 1, f"Alternative {idx + 1}"))

        # 1. Dessiner d'abord les routes inactives en gris
        for wps, r_idx, label in routes:
            if r_idx != selected_route_idx:
                coords = [[wp.lat, wp.lon] for wp in wps]
                folium.PolyLine(
                    locations=coords,
                    color=COLOR_ROUTE_ALT,
                    weight=4,
                    opacity=0.6,
                    tooltip=f"{label} (Inactif)",
                    dash_array="10 5",
                ).add_to(m)

        # 2. Dessiner la route active selectionnee en bleu
        for wps, r_idx, label in routes:
            if r_idx == selected_route_idx:
                coords = [[wp.lat, wp.lon] for wp in wps]
                folium.PolyLine(
                    locations=coords,
                    color=COLOR_ROUTE_MAIN,
                    weight=6,
                    opacity=0.9,
                    tooltip=f"{label} (Actif)",
                ).add_to(m)

                # Dessiner des cercles interactifs pour CHAQUE waypoint de la route active
                for i, wp in enumerate(wps):
                    if 0 < i < len(wps) - 1:
                        idx_a = wp.index_aller if wp.index_aller is not None else (i + 1)
                        if getattr(wp, "passages", None):
                            passages_str = ", ".join(str(p) for p in wp.passages)
                            tooltip = f"Waypoint traverse {len(wp.passages)} fois (passages : {passages_str})"
                        elif wp.index_retour is None:
                            tooltip = f"Waypoint #{idx_a}"
                        else:
                            idx_r = wp.index_retour
                            tooltip = f"Waypoint #{idx_a} (Aller: {idx_a} | Retour: {idx_r})"
                        folium.CircleMarker(
                            location=[wp.lat, wp.lon],
                            radius=4,
                            color=COLOR_ROUTE_MAIN,
                            fill=True,
                            fill_color="white",
                            fill_opacity=0.9,
                            tooltip=tooltip,
                        ).add_to(m)

    # --- Sauvegarde en fichier HTML complet (standalone) ---
    m.save(str(MAP_HTML_FILE))

    # --- Injection du JS QWebChannel dans le HTML sauvegarde ---
    # On lit le fichier, on injecte le script, on reecrit.
    html = MAP_HTML_FILE.read_text(encoding="utf-8")

    webchannel_js = """
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<script>
  var pyBridge = null;

  /* Connexion au bridge Python avec retry toutes les 300ms */
  function connectBridge() {
    if (typeof qt !== 'undefined' && qt.webChannelTransport) {
      new QWebChannel(qt.webChannelTransport, function(channel) {
        pyBridge = channel.objects.bridge;
      });
    } else {
      setTimeout(connectBridge, 300);
    }
  }

  /* Accrochage des clics sur la carte Leaflet */
  function hookLeafletClick() {
    for (var key in window) {
      if (key.startsWith('map_') && window[key] &&
          typeof window[key].on === 'function') {
        window[key].on('click', function(e) {
          if (pyBridge) {
            pyBridge.on_map_click(e.latlng.lat, e.latlng.lng);
          }
        });
        break;
      }
    }
  }

  window.addEventListener('load', function() {
    connectBridge();
    setTimeout(hookLeafletClick, 1200);
  });
</script>
"""
    html = html.replace("</body>", webchannel_js + "\n</body>")
    MAP_HTML_FILE.write_text(html, encoding="utf-8")


# ===========================================================================
# Bridge JavaScript <-> Python (QWebChannel)
# ===========================================================================

class MapBridge(QObject):
    """
    Pont de communication entre la carte Leaflet.js et Python.

    Quand l'utilisateur clique sur la carte, JavaScript appelle
    on_map_click() via QWebChannel, ce qui emet le signal map_clicked.
    """
    map_clicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)

    @pyqtSlot(float, float)
    def on_map_click(self, lat: float, lon: float):
        """Slot appele depuis JavaScript a chaque clic sur la carte."""
        self.map_clicked.emit(lat, lon)


# ===========================================================================
# Popup d'erreur personnalisee
# ===========================================================================

class ErrorPopup(QDialog):
    """Fenetre popup modale d'erreur avec style PGuard."""

    def __init__(self, title: str, message: str, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(True)
        self.setMinimumWidth(420)
        self.setWindowFlag(Qt.WindowType.WindowContextHelpButtonHint, False)
        self._build_ui(title, message)
        self.setStyleSheet(f"""
            QDialog {{
                background-color: {COLOR_BG_CARD};
                border: 1px solid {COLOR_DANGER};
                border-radius: 12px;
            }}
        """)

    def _build_ui(self, title: str, message: str):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 20)
        layout.setSpacing(16)

        # Titre
        h = QHBoxLayout()
        ico = QLabel("!")
        ico.setFont(QFont("Segoe UI", 22, QFont.Weight.Bold))
        ico.setStyleSheet(f"color:{COLOR_DANGER}; padding:0 8px;")
        h.addWidget(ico)
        t = QLabel(title)
        t.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
        t.setStyleSheet(f"color:{COLOR_DANGER};")
        t.setWordWrap(True)
        h.addWidget(t, stretch=1)
        layout.addLayout(h)

        # Separateur
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"border:1px solid {COLOR_BORDER};")
        layout.addWidget(sep)

        # Message
        msg = QLabel(message)
        msg.setFont(QFont("Segoe UI", 11))
        msg.setStyleSheet(f"color:{COLOR_TEXT_PRIMARY};")
        msg.setWordWrap(True)
        layout.addWidget(msg)

        # Bouton fermer
        btn = QPushButton("  X  Fermer")
        btn.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        btn.setFixedHeight(40)
        btn.clicked.connect(self.accept)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color:{COLOR_DANGER}; color:white;
                border:none; border-radius:8px; padding:0 20px;
            }}
            QPushButton:hover {{ background-color:#FF6B6B; }}
        """)
        row = QHBoxLayout()
        row.addStretch()
        row.addWidget(btn)
        layout.addLayout(row)


# ===========================================================================
# Fenetre principale
# ===========================================================================

class PGuardNavigatorWindow(QMainWindow):
    """
    Fenetre principale de PGuard Navigator.

    Gere l'affichage de la carte, les clics, le calcul de route,
    le bouton Retour (undo), l'export JSON et les erreurs.
    """

    def __init__(self, osm_file: str, http_port: int):
        """
        Args:
            osm_file  : Chemin du fichier OSM.
            http_port : Port du serveur HTTP local servant la carte.
        """
        super().__init__()
        self.osm_file  = osm_file
        self.http_port = http_port

        # --- Etat de l'application ---
        self._start_wp:     Waypoint | None     = None
        self._end_wp:       Waypoint | None     = None
        self._via_wps:      list[Waypoint]      = []
        self._n_via_target: int                 = 0
        self._route_result: RouteResult | None  = None
        self._undo_stack:   list[tuple]         = []
        self.selected_route_idx: int            = 0

        # --- Moteur de routage ---
        try:
            self.engine = PGuardRouteEngine(osm_file)
        except Exception as e:
            QMessageBox.critical(self, "Erreur OSM", str(e))
            sys.exit(1)

        bounds = self.engine.bounds
        self._map_center_lat = (bounds["minlat"] + bounds["maxlat"]) / 2
        self._map_center_lon = (bounds["minlon"] + bounds["maxlon"]) / 2

        # --- Construction UI ---
        self._setup_window()
        self._build_ui()
        self._apply_stylesheet()
        self._setup_web_channel()

        # --- Premiere carte ---
        self._refresh_map()
        self.status_bar.showMessage(
            "Carte chargee. Cliquez pour selectionner le point de DEPART (vert)."
        )

    # ------------------------------------------------------------------
    # Configuration fenetre
    # ------------------------------------------------------------------

    def _setup_window(self):
        self.setWindowTitle("PGuard Navigator  -  Enova Robotics")
        self.setMinimumSize(1200, 750)
        self.resize(1440, 860)
        self.status_bar = QStatusBar()
        self.status_bar.setFont(QFont("Segoe UI", 10))
        self.setStatusBar(self.status_bar)

    # ------------------------------------------------------------------
    # Construction de l'UI
    # ------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main = QHBoxLayout(central)
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)

        main.addWidget(self._build_side_panel())

        # Zone carte
        map_w = QWidget()
        map_l = QVBoxLayout(map_w)
        map_l.setContentsMargins(0, 0, 0, 0)
        self.web_view = QWebEngineView()
        self.web_view.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        map_l.addWidget(self.web_view)
        main.addWidget(map_w, stretch=1)

    def _build_side_panel(self) -> QWidget:
        panel = QWidget()
        panel.setObjectName("side_panel")
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(12)

        # Logo
        hdr = QLabel("PGuard\nNavigator")
        hdr.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hdr.setFont(QFont("Segoe UI", 18, QFont.Weight.Bold))
        hdr.setStyleSheet(f"color:{COLOR_TEXT_PRIMARY}; padding:12px 0;")
        lay.addWidget(hdr)

        sub = QLabel("Enova Robotics  -  Navigation PGuard")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setFont(QFont("Segoe UI", 9))
        sub.setStyleSheet(f"color:{COLOR_TEXT_SECONDARY};")
        lay.addWidget(sub)

        lay.addWidget(self._sep())

        # Info OSM
        info = QLabel(
            f"Fichier OSM : {Path(self.osm_file).name}\n"
            f"Zone : lat [{self.engine.bounds['minlat']:.4f}, "
            f"{self.engine.bounds['maxlat']:.4f}]\n"
            f"       lon [{self.engine.bounds['minlon']:.4f}, "
            f"{self.engine.bounds['maxlon']:.4f}]\n"
            f"Noeuds : {self.engine.graph.number_of_nodes()}\n"
            f"Aretes : {self.engine.graph.number_of_edges()}"
        )
        info.setFont(QFont("Segoe UI", 9))
        info.setStyleSheet(
            f"color:{COLOR_TEXT_SECONDARY}; background:{COLOR_BG_CARD};"
            f"border:1px solid {COLOR_BORDER}; border-radius:8px; padding:10px;"
        )
        info.setWordWrap(True)
        lay.addWidget(info)

        lay.addWidget(self._sep())

        # Points
        lay.addWidget(self._section("Points de navigation"))
        self.lbl_start = self._status_card("Depart", "Non defini", COLOR_TEXT_SECONDARY)
        lay.addWidget(self.lbl_start)
        self.lbl_via = self._status_card("Points intermediaires", "Aucun", COLOR_TEXT_SECONDARY)
        lay.addWidget(self.lbl_via)
        self.lbl_end = self._status_card("Arrivee", "Non defini", COLOR_TEXT_SECONDARY)
        lay.addWidget(self.lbl_end)

        # Trajectoire
        lay.addWidget(self._sep())
        lay.addWidget(self._section("Trajectoire calculee"))
        self.lbl_dist  = self._status_card("Distance",    "-", COLOR_TEXT_SECONDARY)
        self.lbl_wpts  = self._status_card("Waypoints",   "-", COLOR_TEXT_SECONDARY)
        lay.addWidget(self.lbl_dist)
        lay.addWidget(self.lbl_wpts)

        # Choix de la route active (Alternative 1, 2, ...)
        self.lbl_alts = QWidget()
        self.lbl_alts.setStyleSheet(f"background:{COLOR_BG_CARD}; border-radius:6px;")
        lay_select = QVBoxLayout(self.lbl_alts)
        lay_select.setContentsMargins(8, 6, 8, 6)
        lay_select.setSpacing(4)
        lbl_title = QLabel("Choix de la Route")
        lbl_title.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        lbl_title.setStyleSheet(f"color:{COLOR_TEXT_PRIMARY};")
        lay_select.addWidget(lbl_title)

        self.cmb_route_select = QComboBox()
        self.cmb_route_select.setFont(QFont("Segoe UI", 9))
        self.cmb_route_select.addItem("Aucun itineraire")
        self.cmb_route_select.setEnabled(False)
        self.cmb_route_select.currentIndexChanged.connect(self._on_route_selection_changed)
        self.cmb_route_select.setStyleSheet(f"""
            QComboBox {{
                background-color: {COLOR_BG_DARK};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                padding: 4px 8px;
                min-height: 24px;
            }}
            QComboBox::drop-down {{
                border: none;
            }}
            QComboBox QAbstractItemView {{
                background-color: {COLOR_BG_CARD};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER};
                selection-background-color: {COLOR_ACCENT};
            }}
        """)
        lay_select.addWidget(self.cmb_route_select)
        lay.addWidget(self.lbl_alts)

        lay.addWidget(self._sep())
        lay.addStretch()

        # Boutons
        lay.addWidget(self._section("Actions"))

        self.btn_generate = self._btn(
            "  Generer la trajectoire", COLOR_ACCENT, COLOR_ACCENT_HOVER
        )
        self.btn_generate.setEnabled(False)
        self.btn_generate.clicked.connect(self._on_generate_route)
        lay.addWidget(self.btn_generate)

        self.btn_undo = self._btn(
            "  Retour (annuler)", COLOR_BG_CARD, COLOR_BORDER,
            text_color=COLOR_TEXT_PRIMARY, border=COLOR_BORDER,
        )
        self.btn_undo.setEnabled(False)
        self.btn_undo.clicked.connect(self._on_undo)
        lay.addWidget(self.btn_undo)

        self.btn_export = self._btn(
            "  Exporter (JSON)", COLOR_INFO, "#4A9EFF"
        )
        self.btn_export.setEnabled(False)
        self.btn_export.clicked.connect(self._on_export_json)
        lay.addWidget(self.btn_export)

        self.btn_reset = self._btn(
            "  Reinitialiser", COLOR_DANGER, "#FF6B6B"
        )
        self.btn_reset.setEnabled(False)
        self.btn_reset.clicked.connect(self._on_reset)
        lay.addWidget(self.btn_reset)

        lay.addSpacing(8)
        foot = QLabel("2025 Enova Robotics  -  Interne")
        foot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        foot.setFont(QFont("Segoe UI", 8))
        foot.setStyleSheet(f"color:{COLOR_BORDER};")
        lay.addWidget(foot)

        # Panneau scrollable : evite que le contenu (variable selon le
        # nombre de points intermediaires) ne soit tronque si la fenetre
        # est trop petite pour tout afficher d'un coup.
        scroll = QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(300)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setObjectName("side_panel_scroll")
        return scroll

    # ------------------------------------------------------------------
    # Widgets utilitaires
    # ------------------------------------------------------------------

    def _sep(self) -> QFrame:
        s = QFrame()
        s.setFrameShape(QFrame.Shape.HLine)
        s.setFixedHeight(1)
        s.setStyleSheet(f"background:{COLOR_BORDER}; border:none;")
        return s

    def _section(self, text: str) -> QLabel:
        l = QLabel(text)
        l.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        l.setStyleSheet(f"color:{COLOR_TEXT_SECONDARY}; padding:4px 0 2px 0;")
        return l

    def _status_card(self, key: str, value: str, color: str) -> QWidget:
        """Widget cle + valeur dans une carte."""
        c = QWidget()
        c.setStyleSheet(f"background:{COLOR_BG_CARD}; border-radius:6px;")
        lay = QVBoxLayout(c)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        k = QLabel(key)
        k.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        k.setStyleSheet(f"color:{COLOR_TEXT_PRIMARY};")
        lay.addWidget(k)
        v = QLabel(value)
        v.setFont(QFont("Courier New", 9))
        v.setStyleSheet(f"color:{color};")
        v.setWordWrap(True)
        lay.addWidget(v)
        c._val = v   # reference pour mise a jour ulterieure
        return c

    def _btn(self, text, bg, hover, text_color="white", border="transparent") -> QPushButton:
        b = QPushButton(text)
        b.setFixedHeight(42)
        b.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        b.setCursor(Qt.CursorShape.PointingHandCursor)
        b.setStyleSheet(f"""
            QPushButton {{
                background:{bg}; color:{text_color};
                border:1px solid {border}; border-radius:8px;
                padding:0 12px; text-align:left;
            }}
            QPushButton:hover {{ background:{hover}; }}
            QPushButton:disabled {{
                background:{COLOR_BG_CARD}; color:{COLOR_TEXT_SECONDARY};
                border:1px solid {COLOR_BORDER};
            }}
        """)
        return b

    # ------------------------------------------------------------------
    # WebChannel
    # ------------------------------------------------------------------

    def _setup_web_channel(self):
        """Configure le pont de communication JS <-> Python."""
        self.bridge = MapBridge(self)
        self.bridge.map_clicked.connect(self._on_map_clicked)
        self.channel = QWebChannel(self.web_view.page())
        self.channel.registerObject("bridge", self.bridge)
        self.web_view.page().setWebChannel(self.channel)

    # ------------------------------------------------------------------
    # Rafraichissement de la carte via HTTP local
    # ------------------------------------------------------------------

    def _refresh_map(self):
        """
        Regenere la carte Folium, la sauvegarde en HTML et la charge
        via le serveur HTTP local pour contourner les restrictions file://.

        URL : http://127.0.0.1:{port}/_map_cache.html
        """
        build_folium_map(
            center_lat=self._map_center_lat,
            center_lon=self._map_center_lon,
            zoom=16,
            start_wp=self._start_wp,
            end_wp=self._end_wp,
            via_wps=self._via_wps,
            route_result=self._route_result,
            selected_route_idx=self.selected_route_idx,
            graph=self.engine.graph,
        )
        url = QUrl(f"http://127.0.0.1:{self.http_port}/{MAP_HTML_FILE.name}?t={int(datetime.now().timestamp())}")
        self.web_view.load(url)

    # ------------------------------------------------------------------
    # Mise a jour des labels
    # ------------------------------------------------------------------

    def _update_labels(self):
        """Met a jour les labels du panneau et l'etat des boutons."""
        if self._start_wp:
            self.lbl_start._val.setText(
                f"{self._start_wp.lat:.6f}, {self._start_wp.lon:.6f}"
            )
            self.lbl_start._val.setStyleSheet(
                f"color:{COLOR_START_MARKER}; font-family:Courier New;"
            )
        else:
            self.lbl_start._val.setText("Non defini")
            self.lbl_start._val.setStyleSheet(
                f"color:{COLOR_TEXT_SECONDARY}; font-family:Courier New;"
            )

        if self._n_via_target > 0:
            self.lbl_via._val.setText(f"{len(self._via_wps)} / {self._n_via_target} definis")
            self.lbl_via._val.setStyleSheet(
                f"color:{COLOR_NAV_NETWORK}; font-family:Courier New;"
            )
        else:
            self.lbl_via._val.setText("Aucun")
            self.lbl_via._val.setStyleSheet(
                f"color:{COLOR_TEXT_SECONDARY}; font-family:Courier New;"
            )

        if self._end_wp:
            self.lbl_end._val.setText(
                f"{self._end_wp.lat:.6f}, {self._end_wp.lon:.6f}"
            )
            self.lbl_end._val.setStyleSheet(
                f"color:{COLOR_END_MARKER}; font-family:Courier New;"
            )
        else:
            self.lbl_end._val.setText("Non defini")
            self.lbl_end._val.setStyleSheet(
                f"color:{COLOR_TEXT_SECONDARY}; font-family:Courier New;"
            )

        if self._route_result:
            r = self._route_result
            
            # Populer le sélecteur si nécessaire
            if self.cmb_route_select.count() <= 1 or "Aucun" in self.cmb_route_select.currentText():
                self.cmb_route_select.blockSignals(True)
                self.cmb_route_select.clear()
                self.cmb_route_select.addItem(f"Route principale ({r.total_distance_m:.1f} m)")
                for idx, alt_wps in enumerate(r.alternatives):
                    alt_dist = calculate_path_distance(alt_wps) * 2.0
                    self.cmb_route_select.addItem(f"Alternative {idx + 1} ({alt_dist:.1f} m)")
                self.cmb_route_select.setCurrentIndex(self.selected_route_idx)
                self.cmb_route_select.setEnabled(True)
                self.cmb_route_select.blockSignals(False)

            # Extraire les infos de la route active sélectionnée
            if self.selected_route_idx == 0:
                active_wps = r.waypoints
                active_dist = r.total_distance_m
            else:
                active_wps = r.alternatives[self.selected_route_idx - 1]
                active_dist = calculate_path_distance(active_wps) * 2.0

            self.lbl_dist._val.setText(
                f"{active_dist:.1f} m  ({active_dist/1000:.3f} km)"
            )
            self.lbl_dist._val.setStyleSheet(
                f"color:{COLOR_ROUTE_MAIN}; font-family:Courier New;"
            )
            self.lbl_wpts._val.setText(str(len(active_wps)))
            self.lbl_wpts._val.setStyleSheet(
                f"color:{COLOR_ROUTE_MAIN}; font-family:Courier New;"
            )
        else:
            self.cmb_route_select.blockSignals(True)
            self.cmb_route_select.clear()
            self.cmb_route_select.addItem("Aucun itineraire")
            self.cmb_route_select.setEnabled(False)
            self.cmb_route_select.blockSignals(False)
            self.selected_route_idx = 0

            for lbl in (self.lbl_dist, self.lbl_wpts):
                lbl._val.setText("-")
                lbl._val.setStyleSheet(
                    f"color:{COLOR_TEXT_SECONDARY}; font-family:Courier New;"
                )

        all_points_set = (
            self._start_wp is not None
            and self._end_wp is not None
            and len(self._via_wps) >= self._n_via_target
        )
        self.btn_generate.setEnabled(all_points_set)
        self.btn_undo.setEnabled(bool(self._undo_stack))
        self.btn_export.setEnabled(self._route_result is not None)
        any_state = (
            self._start_wp is not None
            or self._end_wp is not None
            or bool(self._via_wps)
            or self._route_result is not None
        )
        self.btn_reset.setEnabled(any_state)

    # ------------------------------------------------------------------
    # Gestionnaires d'evenements
    # ------------------------------------------------------------------

    def _ask_via_count(self) -> int:
        """
        Demande a l'utilisateur combien de points intermediaires il souhaite
        imposer sur le trajet, juste apres avoir pose le point de depart.
        """
        n, ok = QInputDialog.getInt(
            self,
            "Points intermediaires",
            "Nombre de points intermediaires a imposer sur le trajet\n"
            "(0 = itineraire direct depart -> arrivee) :",
            0, 0, 20, 1,
        )
        return n if ok else 0

    def _on_map_clicked(self, lat: float, lon: float):
        """
        Clic sur la carte Leaflet.
        Ordre : depart -> (N points intermediaires, si demandes) -> arrivee.
        """
        if self._route_result is not None:
            self.status_bar.showMessage(
                "Route deja calculee. Utilisez 'Retour' pour recommencer."
            )
            return

        if not self.engine.is_within_bounds(lat, lon):
            ErrorPopup(
                "Zone hors carte",
                f"Le point ({lat:.5f}, {lon:.5f}) est en dehors de la zone OSM.\n"
                "Veuillez cliquer a l'interieur de la zone cartographiee.",
                self,
            ).exec()
            return

        # Vérifier l'accessibilité (distance à la route la plus proche <= 5.0 mètres)
        snapped_lat, snapped_lon, dist_to_road = self.engine.nearest_point_on_road(lat, lon)
        if dist_to_road > 5.0:
            ErrorPopup(
                "Point non accessible",
                f"Le point ({lat:.5f}, {lon:.5f}) est à {dist_to_road:.1f} m d'une voie navigable.\n"
                "Le robot PGuard ne peut circuler que sur les routes ou chemins autorisés.\n"
                "Veuillez cliquer plus près d'une voie.",
                self,
            ).exec()
            return

        # Accrocher le point exactement sur la route (le clic peut être à
        # quelques mètres à côté visuellement, même quand il est accepté).
        lat, lon = snapped_lat, snapped_lon

        if self._start_wp is None:
            self._undo_stack.append(("clear_start", None))
            self._start_wp = Waypoint(lat=lat, lon=lon)
            self._n_via_target = self._ask_via_count()
            if self._n_via_target > 0:
                self.status_bar.showMessage(
                    f"Depart selectionne : ({lat:.5f}, {lon:.5f})  -  "
                    f"Cliquez pour selectionner le point intermediaire "
                    f"1/{self._n_via_target}."
                )
            else:
                self.status_bar.showMessage(
                    f"Depart selectionne : ({lat:.5f}, {lon:.5f})  -  "
                    "Cliquez pour selectionner le point d'ARRIVEE."
                )
        elif len(self._via_wps) < self._n_via_target:
            self._undo_stack.append(("clear_via", None))
            self._via_wps.append(Waypoint(lat=lat, lon=lon))
            done = len(self._via_wps)
            if done < self._n_via_target:
                self.status_bar.showMessage(
                    f"Point intermediaire {done}/{self._n_via_target} selectionne : "
                    f"({lat:.5f}, {lon:.5f})  -  "
                    f"Cliquez pour selectionner le point intermediaire "
                    f"{done + 1}/{self._n_via_target}."
                )
            else:
                self.status_bar.showMessage(
                    f"Point intermediaire {done}/{self._n_via_target} selectionne : "
                    f"({lat:.5f}, {lon:.5f})  -  "
                    "Cliquez pour selectionner le point d'ARRIVEE."
                )
        elif self._end_wp is None:
            self._undo_stack.append(("clear_end", None))
            self._end_wp = Waypoint(lat=lat, lon=lon)
            self.status_bar.showMessage(
                f"Arrivee selectionnee : ({lat:.5f}, {lon:.5f})  -  "
                "Cliquez sur 'Generer la trajectoire' pour calculer l'itineraire."
            )
        else:
            self.status_bar.showMessage(
                "Tous les points sont definis. Cliquez 'Generer' ou 'Retour'."
            )
            return

        self._refresh_map()
        self._update_labels()

    def _on_generate_route(self):
        """Calcule et affiche l'itineraire entre depart et arrivee (via les points intermediaires eventuels)."""
        if not self._start_wp or not self._end_wp:
            return
        if len(self._via_wps) < self._n_via_target:
            return

        self.status_bar.showMessage("Calcul de l'itineraire en cours ...")
        QApplication.processEvents()

        try:
            if self._via_wps:
                all_points = (
                    [Waypoint(lat=self._start_wp.lat, lon=self._start_wp.lon)]
                    + [Waypoint(lat=wp.lat, lon=wp.lon) for wp in self._via_wps]
                    + [Waypoint(lat=self._end_wp.lat, lon=self._end_wp.lon)]
                )
                result = self.engine.compute_route_via(all_points, algorithm="astar")
            else:
                result = self.engine.compute_route(
                    start=Waypoint(lat=self._start_wp.lat, lon=self._start_wp.lon),
                    end=Waypoint(lat=self._end_wp.lat,   lon=self._end_wp.lon),
                    algorithm="astar",
                    nb_alternatives=2,
                )
            self._undo_stack.append(("clear_route", None))
            self._route_result = result
            self.selected_route_idx = 0
            self.status_bar.showMessage(
                f"Trajectoire calculee : {result.total_distance_m:.0f} m  -  "
                f"{len(result.waypoints)} waypoints  -  "
                f"{len(result.alternatives)} alternative(s)."
            )
        except ValueError as e:
            ErrorPopup("Impossible de calculer l'itineraire", str(e), self).exec()
            self.status_bar.showMessage(f"Erreur : {e}")
            return
        except Exception as e:
            ErrorPopup("Erreur inattendue", str(e), self).exec()
            return

        self._refresh_map()
        self._update_labels()

    @pyqtSlot(int)
    def _on_route_selection_changed(self, index: int):
        """Appele quand l'utilisateur change de route dans le selecteur."""
        if index < 0 or not self._route_result:
            return
        self.selected_route_idx = index
        self._refresh_map()
        self._update_labels()

    def _on_undo(self):
        """Annule la derniere action (LIFO : route -> arrivee -> points intermediaires -> depart)."""
        if not self._undo_stack:
            return
        action, _ = self._undo_stack.pop()
        if action == "clear_route":
            self._route_result = None
            self.status_bar.showMessage("Route supprimee.")
        elif action == "clear_end":
            self._end_wp = None
            self.status_bar.showMessage("Point d'arrivee supprime.")
        elif action == "clear_via":
            if self._via_wps:
                self._via_wps.pop()
            self.status_bar.showMessage("Point intermediaire supprime.")
        elif action == "clear_start":
            self._start_wp = None
            self._n_via_target = 0
            self.status_bar.showMessage("Point de depart supprime.")
        self._refresh_map()
        self._update_labels()

    def _on_reset(self):
        """Reinitialise completement l'application : supprime depart, arrivee, route et historique."""
        self._start_wp     = None
        self._end_wp       = None
        self._via_wps      = []
        self._n_via_target = 0
        self._route_result = None
        self._undo_stack   = []
        self.selected_route_idx = 0
        self.status_bar.showMessage(
            "Reinitialisation complete. Cliquez pour selectionner le point de DEPART."
        )
        self._refresh_map()
        self._update_labels()

    def _on_export_json(self):
        """Exporte les waypoints dans un fichier JSON."""
        if not self._route_result:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self, "Exporter la trajectoire PGuard",
            str(Path.home() / f"pguard_trajectory_{ts}.json"),
            "Fichiers JSON (*.json)",
        )
        if not path:
            return
        try:
            self.engine.export_json(self._route_result, path)
            self.status_bar.showMessage(f"Trajectoire exportee : {path}")
            QMessageBox.information(
                self, "Export reussi",
                f"Trajectoire exportee !\n\nFichier : {path}\n"
                f"Waypoints : {len(self._route_result.waypoints)}\n"
                f"Distance : {self._route_result.total_distance_m:.1f} m",
            )
        except Exception as e:
            ErrorPopup("Erreur d'export", str(e), self).exec()

    # ------------------------------------------------------------------
    # Feuille de style globale
    # ------------------------------------------------------------------

    def _apply_stylesheet(self):
        self.setStyleSheet(f"""
            QMainWindow {{ background:{COLOR_BG_DARK}; }}
            #side_panel {{
                background:{COLOR_BG_PANEL};
            }}
            #side_panel_scroll {{
                background:{COLOR_BG_PANEL};
                border:none;
                border-right:1px solid {COLOR_BORDER};
            }}
            #side_panel_scroll > QWidget > QWidget {{
                background:{COLOR_BG_PANEL};
            }}
            QStatusBar {{
                background:{COLOR_BG_PANEL};
                color:{COLOR_TEXT_SECONDARY};
                border-top:1px solid {COLOR_BORDER};
                padding:4px 8px;
            }}
            QMessageBox {{
                background:{COLOR_BG_CARD}; color:{COLOR_TEXT_PRIMARY};
            }}
            QMessageBox QPushButton {{
                background:{COLOR_ACCENT}; color:white;
                border:none; border-radius:6px;
                padding:6px 20px; min-width:80px;
            }}
        """)


# ===========================================================================
# Point d'entree
# ===========================================================================

def main():
    """Lance l'application PGuard Navigator."""
    import argparse
    parser = argparse.ArgumentParser(
        description="PGuard Navigator - Interface de navigation"
    )
    parser.add_argument(
        "--osm",
        default=str(DEFAULT_OSM_FILE),
        help="Chemin du fichier OSM",
    )
    args = parser.parse_args()

    if not Path(args.osm).exists():
        print(f"[ERREUR] Fichier OSM introuvable : {args.osm}")
        sys.exit(1)

    # --- Demarrer le serveur HTTP local ---
    # Sert le repertoire du projet via http://127.0.0.1:PORT/
    project_dir = str(_application_dir())
    http_port = start_local_http_server(project_dir)

    # --- Creer l'application PyQt6 ---
    app = QApplication(sys.argv)
    app.setApplicationName("PGuard Navigator")
    app.setOrganizationName("Enova Robotics")
    app.setApplicationVersion("2.0.0")
    app.setFont(QFont("Segoe UI", 10))

    # --- Lancer la fenetre principale ---
    window = PGuardNavigatorWindow(osm_file=args.osm, http_port=http_port)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
