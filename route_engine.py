"""
==============================================================================
 route_engine.py  –  Moteur de génération de trajectoire pour PGuard
==============================================================================
 Auteur  : Stagiaire Enova Robotics
 Version : 1.0.0
 Projet  : Robot de sécurité PGuard – Navigation autonome

 Description :
     Ce module constitue le BACK-END de calcul d'itinéraire.
     Il charge un fichier OpenStreetMap (.osm), construit un graphe routier
     filtré (seules les voies navigables sont conservées), puis calcule le
     chemin le plus court entre deux points GPS via l'algorithme A* ou
     Dijkstra fourni par NetworkX.

     Points clés de sécurité :
       - Seules les arêtes de type highway navigable sont intégrées au graphe.
       - Les zones vertes, bâtiments et aménagements non-routiers sont exclus.
       - Le graphe est non-orienté (le robot peut se déplacer dans les deux sens).

 Dépendances :
     osmnx, networkx, shapely, pyproj  (voir requirements.txt)
==============================================================================
"""

import math
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional

import networkx as nx
from shapely.geometry import LineString, Polygon
from pyproj import Transformer

# ---------------------------------------------------------------------------
# Constantes de filtrage des tags OSM (CRITIQUE pour la sécurité du robot)
# ---------------------------------------------------------------------------

# Tags highway autorisés : uniquement les vraies routes carrossables sur
# lesquelles un robot peut circuler. Les trottoirs, pistes cyclables,
# chemins piétons et zones piétonnes sont volontairement exclus — trop
# souvent non revêtus ou impraticables pour un robot (ex: chemin en
# pointillé traversant un parc), même s'ils sont tagués "navigables" dans OSM.
ALLOWED_HIGHWAY_TAGS = {
    "trunk",           # Grande route principale
    "trunk_link",      # Bretelle de grande route
    "primary",         # Route principale
    "primary_link",
    "secondary",       # Route secondaire
    "secondary_link",
    "tertiary",        # Route tertiaire
    "tertiary_link",
    "residential",     # Rue résidentielle
    "service",         # Voie de service / parking / accès
    "unclassified",    # Route non classifiée mais carrossable
    "living_street",   # Zone de rencontre (vitesse réduite)
    "road",            # Route de type indéfini
}

# Tags highway INTERDITS : bâtiments, escaliers, ascenseurs, etc.
FORBIDDEN_HIGHWAY_TAGS = {
    "steps",           # Escaliers — le robot ne peut pas les franchir
    "elevator",        # Ascenseur
    "construction",    # En construction
    "proposed",        # Proposé mais non construit
    "abandoned",       # Abandonné
    "disused",         # Inutilisé
}

# Sous-tags "service" INTERDITS : ce ne sont pas des voies de circulation
# réelles (allées entre places de parking, etc.), même si highway=service
# est par ailleurs autorisé.
FORBIDDEN_SERVICE_TAGS = {
    "parking_aisle",   # Allée de stationnement — pas une route carrossable
}

# Identifiants de ways OSM exclus individuellement (à compléter au cas par
# cas si une géométrie s'avère non fiable malgré un tag valide).
EXCLUDED_WAY_IDS: set[int] = set()

# Tags de zone interdite : le robot ne doit PAS traverser ces surfaces.
# Ces tags apparaissent souvent sur des ways/relations (pas des edges routiers).
FORBIDDEN_ZONE_TAGS = {
    "leisure":  {"park", "garden", "pitch", "playground", "golf_course"},
    "landuse":  {"grass", "forest", "meadow", "cemetery", "farmland",
                 "recreation_ground", "village_green"},
    "natural":  {"wood", "scrub", "heath", "grassland", "water", "wetland",
                 "beach", "sand", "cliff", "peak"},
    "building": None,   # None signifie : toute valeur est interdite
    "amenity":  {"restaurant", "cafe", "bar", "fast_food", "food_court"},
}

# Rayon en mètres pour trouver le nœud du graphe le plus proche d'un clic GPS
DEFAULT_SNAP_RADIUS_M = 200.0

# Distance maximale (en mètres) entre deux points pour les considérer comme
# "confondus" (même endroit). Utilisé en plus du partage du même nœud le
# plus proche, pour ne pas traiter à tort deux points éloignés comme
# confondus simplement parce que le réseau routier est peu couvert autour
# d'eux (ce qui produirait une trajectoire hors-route non désirée).
CONFOUNDED_POINTS_MAX_M = 15.0

# ---------------------------------------------------------------------------
# Structures de données
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    """Représente un point de passage GPS."""
    lat: float
    lon: float
    node_id: Optional[int] = None   # Identifiant OSM du nœud (si disponible)
    index_aller: Optional[int] = None
    index_retour: Optional[int] = None
    # Rempli uniquement si ce point physique est traversé plus de deux fois
    # (ex: une boucle qui recroise son propre tracé) : liste tous les
    # numéros de passage (aller + retour, toutes occurrences confondues).
    passages: Optional[list[int]] = None

    def to_dict(self) -> dict:
        """Convertit le waypoint en dictionnaire JSON-sérialisable."""
        res = {
            "latitude":  self.lat,
            "longitude": self.lon,
        }
        if self.node_id is not None:
            res["node_id"] = self.node_id
        if self.index_aller is not None:
            res["index_aller"] = self.index_aller
        if self.index_retour is not None:
            res["index_retour"] = self.index_retour
        if self.passages is not None:
            res["passages"] = self.passages
        return res

    def __repr__(self):
        return f"Waypoint(lat={self.lat:.6f}, lon={self.lon:.6f}, aller={self.index_aller}, retour={self.index_retour}, passages={self.passages})"


@dataclass
class RouteResult:
    """Résultat complet d'un calcul d'itinéraire."""
    waypoints:       list[Waypoint]       # Séquence de waypoints GPS (aller)
    total_distance_m: float               # Distance totale en mètres
    path_node_ids:   list[int]            # IDs des nœuds OSM traversés
    alternatives:    list[list[Waypoint]] = field(default_factory=list)
    one_way:         bool = False         # True = aller simple (sans retour)

    def to_dict(self) -> dict:
        """Exporte le résultat en dictionnaire structuré (format JSON PGuard)."""
        n_wps = len(self.waypoints)
        return {
            "metadata": {
                "total_waypoints_aller": n_wps,
                "total_waypoints_aller_retour": n_wps if self.one_way else max(0, n_wps * 2 - 1),
                "one_way": self.one_way,
                "total_distance_m":   round(self.total_distance_m, 2),
                "total_distance_km":  round(self.total_distance_m / 1000, 4),
                "nb_alternatives":    len(self.alternatives),
            },
            "trajectory": [wp.to_dict() for wp in self.waypoints],
            "alternatives": [
                [wp.to_dict() for wp in alt]
                for alt in self.alternatives
            ],
        }


# ---------------------------------------------------------------------------
# Fonctions utilitaires de géodésie
# ---------------------------------------------------------------------------

def haversine_distance(lat1: float, lon1: float,
                       lat2: float, lon2: float) -> float:
    """
    Calcule la distance en mètres entre deux points GPS (formule de Haversine).

    Args:
        lat1, lon1 : Coordonnées du premier point (degrés décimaux).
        lat2, lon2 : Coordonnées du second point (degrés décimaux).

    Returns:
        Distance en mètres (float).
    """
    R = 6_371_000.0          # Rayon moyen de la Terre en mètres
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi   = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Cap (bearing) en degrés du segment [P1, P2] : 0° = nord, sens horaire."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return math.degrees(math.atan2(x, y)) % 360.0


def _turn_angle_deg(bearing_in: float, bearing_out: float) -> float:
    """Écart angulaire (0-180°) entre deux caps."""
    diff = abs(bearing_out - bearing_in) % 360.0
    return min(diff, 360.0 - diff)


def _quantize_turn_deg(delta_deg: float) -> int:
    """
    Ramène un changement de cap continu au multiple de 90° le plus proche
    dans [-180, 180] (0 = tout droit, ±90 = virage gauche/droite, 180 = demi-tour).
    """
    d = ((delta_deg + 180.0) % 360.0) - 180.0
    q = int(round(d / 90.0)) * 90
    return -180 if q == -180 else q if q != 180 else 180


def _local_xy_projector(origin_lat: float, origin_lon: float):
    """
    Construit une fonction (lat, lon) -> (x, y) en mètres, projection
    azimutale équidistante locale centrée sur (origin_lat, origin_lon)
    (nord = +y, est = +x). Utilisée pour les coordonnées locales du robot
    dans l'export JSON.
    """
    transformer = Transformer.from_crs(
        "EPSG:4326",
        {"proj": "aeqd", "lat_0": origin_lat, "lon_0": origin_lon,
         "datum": "WGS84", "units": "m"},
        always_xy=True,
    )

    def to_xy(lat: float, lon: float) -> tuple[float, float]:
        x, y = transformer.transform(lon, lat)
        return x, y

    return to_xy


def _local_turn_angle(waypoints: list[Waypoint], i: int, window_m: float) -> float:
    """
    Changement de cap au sommet i, lissé sur une fenêtre de `window_m`
    mètres avant/après (au lieu du segment immédiatement adjacent).

    Une route composée de nombreux petits segments rapprochés (digitisation
    OSM imprécise) peut sembler "tourner" légèrement à chaque sommet même
    si elle est globalement droite. Lisser sur une distance plutôt que sur
    un seul segment filtre ce bruit tout en détectant les vrais virages
    (qui montrent un changement de cap net, même sur une fenêtre large).
    """
    n = len(waypoints)

    j, acc = i, 0.0
    while j > 0 and acc < window_m:
        acc += haversine_distance(
            waypoints[j - 1].lat, waypoints[j - 1].lon, waypoints[j].lat, waypoints[j].lon
        )
        j -= 1
    if j == i:
        return 0.0

    k, acc = i, 0.0
    while k < n - 1 and acc < window_m:
        acc += haversine_distance(
            waypoints[k].lat, waypoints[k].lon, waypoints[k + 1].lat, waypoints[k + 1].lon
        )
        k += 1
    if k == i:
        return 0.0

    bearing_in = bearing_deg(waypoints[j].lat, waypoints[j].lon, waypoints[i].lat, waypoints[i].lon)
    bearing_out = bearing_deg(waypoints[i].lat, waypoints[i].lon, waypoints[k].lat, waypoints[k].lon)
    return _turn_angle_deg(bearing_in, bearing_out)


def interpolate_waypoints(
    waypoints: list[Waypoint],
    max_dist: float = 10.0,
    turn_max_dist: float = 3.0,
    turn_angle_deg: float = 50.0,
    turn_window_m: float = 8.0,
) -> list[Waypoint]:
    """
    Interpole la trajectoire pour s'assurer que la distance entre deux
    waypoints successifs ne dépasse jamais max_dist (10 mètres par défaut).

    Aux changements de direction (un sommet où le cap change de plus de
    turn_angle_deg, mesuré en lissant sur turn_window_m mètres pour ignorer
    le bruit de digitisation), les segments adjacents sont densifiés avec
    un espacement plus fin (turn_max_dist, 3 mètres par défaut), pour mieux
    représenter le virage. Les tronçons alignés (ligne droite) gardent
    l'espacement standard, plus large.
    """
    if not waypoints:
        return []

    # Détecte les sommets de virage (nécessite au moins 3 points réels)
    is_turn = [False] * len(waypoints)
    for i in range(1, len(waypoints) - 1):
        if _local_turn_angle(waypoints, i, turn_window_m) >= turn_angle_deg:
            is_turn[i] = True

    interpolated = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        w1 = waypoints[i]
        w2 = waypoints[i+1]

        # Segment plus finement échantillonné s'il touche un virage
        seg_max_dist = turn_max_dist if (is_turn[i] or is_turn[i + 1]) else max_dist

        dist = haversine_distance(w1.lat, w1.lon, w2.lat, w2.lon)
        if dist > seg_max_dist:
            num_segments = math.ceil(dist / seg_max_dist)
            for j in range(1, num_segments):
                fraction = j / num_segments
                interp_lat = w1.lat + fraction * (w2.lat - w1.lat)
                interp_lon = w1.lon + fraction * (w2.lon - w1.lon)
                interpolated.append(Waypoint(lat=interp_lat, lon=interp_lon, node_id=None))
        interpolated.append(w2)
    return interpolated


def calculate_path_distance(waypoints: list[Waypoint]) -> float:
    """
    Calcule la distance totale d'une liste de waypoints GPS.
    """
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += haversine_distance(waypoints[i].lat, waypoints[i].lon,
                                    waypoints[i+1].lat, waypoints[i+1].lon)
    return total


# ---------------------------------------------------------------------------
# Classe principale : Moteur de routage
# ---------------------------------------------------------------------------

class PGuardRouteEngine:
    """
    Moteur de calcul d'itinéraire pour le robot de sécurité PGuard.

    Workflow :
        1. Charger le fichier OSM           → load_osm()
        2. Construire le graphe filtré      → (automatique dans load_osm)
        3. Calculer l'itinéraire            → compute_route()
        4. Exporter les waypoints en JSON   → export_json()

    Exemple d'utilisation :
        engine = PGuardRouteEngine("map.osm")
        result = engine.compute_route(
            start=Waypoint(lat=35.8180, lon=10.5920),
            end=Waypoint(lat=35.8210, lon=10.5945),
        )
        engine.export_json(result, "trajectory.json")
    """

    def __init__(self, osm_file: str):
        """
        Initialise le moteur et charge le fichier OSM.

        Args:
            osm_file : Chemin vers le fichier .osm à analyser.
        """
        self.osm_file = osm_file
        self.graph: nx.Graph = nx.Graph()

        # Dictionnaire node_id → (lat, lon)
        self._nodes: dict[int, tuple[float, float]] = {}

        # Compteur pour les nœuds virtuels temporaires (accrochage précis
        # sur une arête plutôt que sur le nœud le plus proche). Ids négatifs
        # pour ne jamais entrer en collision avec un id OSM réel.
        self._virtual_node_counter: int = 0
        self._active_virtual_nodes: list[tuple] = []

        # Polygones (lon, lat) des zones interdites (bâtiments, espaces verts, ...)
        # Utilisés pour vérifier qu'un trajet direct (hors graphe routier) ne
        # traverse pas une zone où le robot n'a pas le droit de circuler.
        self._forbidden_zone_polygons: list[Polygon] = []

        # Bounds de la carte OSM (pour validation des clics utilisateur)
        self.bounds: dict = {
            "minlat": None, "maxlat": None,
            "minlon": None, "maxlon": None,
        }

        print(f"[PGuard Route Engine] Chargement de '{osm_file}' ...")
        self.load_osm(osm_file)
        print(f"[PGuard Route Engine] Graphe construit : "
              f"{self.graph.number_of_nodes()} nœuds, "
              f"{self.graph.number_of_edges()} arêtes navigables.")

    # ------------------------------------------------------------------
    # Chargement et parsing du fichier OSM
    # ------------------------------------------------------------------

    def load_osm(self, osm_file: str) -> None:
        """
        Parse le fichier OSM et construit le graphe routier filtré.

        Étapes :
          1. Lire les <bounds> pour connaître l'étendue géographique.
          2. Charger tous les <node> (id, lat, lon).
          3. Charger les <way> et ne garder que ceux avec un tag highway
             autorisé ET sans tag de zone interdite.
          4. Pour chaque way valide, créer des arêtes dans le graphe
             pondérées par la distance haversine.

        Args:
            osm_file : Chemin du fichier .osm.
        """
        tree = ET.parse(osm_file)
        root = tree.getroot()

        # 1. Lecture des bounds
        bounds_el = root.find("bounds")
        if bounds_el is not None:
            self.bounds = {
                "minlat": float(bounds_el.get("minlat", 0)),
                "maxlat": float(bounds_el.get("maxlat", 0)),
                "minlon": float(bounds_el.get("minlon", 0)),
                "maxlon": float(bounds_el.get("maxlon", 0)),
            }
            print(f"[OSM] Zone couverte : "
                  f"lat [{self.bounds['minlat']}, {self.bounds['maxlat']}], "
                  f"lon [{self.bounds['minlon']}, {self.bounds['maxlon']}]")

        # 2. Chargement de tous les nœuds
        for node in root.findall("node"):
            nid = int(node.get("id"))
            lat = float(node.get("lat"))
            lon = float(node.get("lon"))
            self._nodes[nid] = (lat, lon)

        print(f"[OSM] {len(self._nodes)} nœuds chargés.")

        # 3 & 4. Traitement des ways
        nb_ways_total    = 0
        nb_ways_accepted = 0

        for way in root.findall("way"):
            nb_ways_total += 1
            tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}

            # --- Capture des zones interdites (indépendant du filtre highway) ---
            if self._is_forbidden_zone(tags):
                zone_refs = [int(nd.get("ref")) for nd in way.findall("nd")]
                self._add_forbidden_zone_polygon(zone_refs)

            # --- Filtre 0 : way exclu individuellement (géométrie non fiable) ---
            if int(way.get("id")) in EXCLUDED_WAY_IDS:
                continue

            # --- Filtre 1 : le way doit avoir un tag highway autorisé ---
            highway_val = tags.get("highway", "")
            if highway_val not in ALLOWED_HIGHWAY_TAGS:
                continue

            # --- Filtre 2 : le way ne doit PAS avoir de tag highway interdit ---
            if highway_val in FORBIDDEN_HIGHWAY_TAGS:
                continue

            # --- Filtre 2bis : sous-tag "service" interdit (ex: allée de parking) ---
            if tags.get("service") in FORBIDDEN_SERVICE_TAGS:
                continue

            # --- Filtre 3 : vérification des tags de zone interdite ---
            if self._is_forbidden_zone(tags):
                continue

            # --- Filtre 4 : exclure les surfaces non-carrossables ---
            surface = tags.get("surface", "")
            if surface in {"sand", "mud", "snow", "ice", "gravel"}:
                # Surfaces difficiles — on les exclut par sécurité
                continue

            # --- Way validé : construction des arêtes ---
            node_refs = [int(nd.get("ref")) for nd in way.findall("nd")]
            nb_ways_accepted += 1

            for i in range(len(node_refs) - 1):
                n1, n2 = node_refs[i], node_refs[i + 1]

                # On ne crée l'arête que si les deux nœuds existent dans notre dict
                if n1 not in self._nodes or n2 not in self._nodes:
                    continue

                lat1, lon1 = self._nodes[n1]
                lat2, lon2 = self._nodes[n2]
                dist = haversine_distance(lat1, lon1, lat2, lon2)

                # Ajout des nœuds avec leurs coordonnées comme attributs
                self.graph.add_node(n1, lat=lat1, lon=lon1)
                self.graph.add_node(n2, lat=lat2, lon=lon2)

                # Arête pondérée par la distance réelle en mètres
                self.graph.add_edge(
                    n1, n2,
                    weight=dist,            # Utilisé par Dijkstra / A*
                    distance_m=dist,
                    highway=highway_val,
                    way_tags=tags,
                )

        print(f"[OSM] Ways traités : {nb_ways_total} total, "
              f"{nb_ways_accepted} acceptés (filtre sécurité appliqué).")
        print(f"[OSM] {len(self._forbidden_zone_polygons)} zones interdites "
              f"capturées pour la vérification des trajets directs.")

    @staticmethod
    def _is_forbidden_zone(tags: dict) -> bool:
        """
        Vérifie si un ensemble de tags OSM correspond à une zone interdite
        (bâtiment, espace vert, zone naturelle, etc.).

        Args:
            tags : Dictionnaire des tags OSM du way.

        Returns:
            True si la zone est interdite (à exclure du graphe).
        """
        for key, forbidden_values in FORBIDDEN_ZONE_TAGS.items():
            if key in tags:
                val = tags[key]
                # forbidden_values=None → toute valeur est interdite (ex: building)
                if forbidden_values is None or val in forbidden_values:
                    return True
        return False

    def _add_forbidden_zone_polygon(self, node_refs: list[int]) -> None:
        """
        Construit un polygone (lon, lat) à partir des nœuds d'un way fermé
        représentant une zone interdite, et le stocke pour les tests de
        croisement (trajet direct hors graphe routier).

        Args:
            node_refs : Références de nœuds du way (ordre du tracé OSM).
        """
        if len(node_refs) < 4 or node_refs[0] != node_refs[-1]:
            return  # Way non fermé : pas exploitable comme polygone de surface
        if any(ref not in self._nodes for ref in node_refs):
            return
        coords = [(self._nodes[ref][1], self._nodes[ref][0]) for ref in node_refs]
        try:
            poly = Polygon(coords)
            if poly.is_valid and poly.area > 0:
                self._forbidden_zone_polygons.append(poly)
        except Exception:
            pass

    def _segment_crosses_forbidden_zone(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> bool:
        """
        Vérifie si le segment direct [P1, P2] traverse une zone interdite
        (bâtiment, espace vert, zone naturelle, etc.).
        """
        if not self._forbidden_zone_polygons:
            return False
        segment = LineString([(lon1, lat1), (lon2, lat2)])
        return any(segment.intersects(poly) for poly in self._forbidden_zone_polygons)

    # ------------------------------------------------------------------
    # Validation géographique
    # ------------------------------------------------------------------

    def is_within_bounds(self, lat: float, lon: float) -> bool:
        """
        Vérifie qu'un point GPS se trouve dans la zone couverte par le fichier OSM.

        Args:
            lat : Latitude du point.
            lon : Longitude du point.

        Returns:
            True si le point est dans la zone OSM.
        """
        if None in self.bounds.values():
            return True  # Pas de bounds → on ne peut pas valider
        margin = 0.005   # Marge de ~500 m pour absorber les imprécisions de clic
        return (
            (self.bounds["minlat"] - margin) <= lat <= (self.bounds["maxlat"] + margin)
            and
            (self.bounds["minlon"] - margin) <= lon <= (self.bounds["maxlon"] + margin)
        )

    # ------------------------------------------------------------------
    # Utilitaire : nœud le plus proche d'un point GPS
    # ------------------------------------------------------------------

    def nearest_node(self, lat: float, lon: float,
                     max_radius_m: float = DEFAULT_SNAP_RADIUS_M
                     ) -> Optional[int]:
        """
        Trouve l'identifiant du nœud du graphe le plus proche d'un point GPS.

        Utilise la distance haversine pour un calcul précis sans projection.
        Renvoie None si aucun nœud n'est trouvé dans le rayon max_radius_m.

        Args:
            lat          : Latitude du point de recherche.
            lon          : Longitude du point de recherche.
            max_radius_m : Rayon maximum de recherche en mètres.

        Returns:
            node_id (int) ou None.
        """
        best_node = None
        best_dist = float("inf")

        for nid in self.graph.nodes:
            n_lat = self.graph.nodes[nid].get("lat")
            n_lon = self.graph.nodes[nid].get("lon")
            if n_lat is None or n_lon is None:
                continue
            d = haversine_distance(lat, lon, n_lat, n_lon)
            if d < best_dist:
                best_dist = d
                best_node = nid

        if best_dist > max_radius_m:
            return None  # Trop loin de tout nœud navigable
        return best_node

    def _snap_point_to_graph(
        self, lat: float, lon: float, max_radius_m: float = DEFAULT_SNAP_RADIUS_M
    ) -> Optional[int]:
        """
        Accroche un point GPS au réseau routier en le projetant sur l'ARÊTE
        navigable la plus proche (pas seulement le nœud le plus proche).

        nearest_node() peut accrocher un point à un nœud situé "de l'autre
        côté" par rapport à la direction du trajet, forçant un aller-retour
        inutile (ex: le nœud le plus proche est 80 m plus loin sur la route
        alors qu'un point à 5 m existe sur l'arête elle-même). En projetant
        sur l'arête, on colle au point réellement le plus proche du réseau.

        Si le point projeté ne coïncide pas avec une extrémité existante,
        un nœud temporaire est inséré en scindant l'arête en deux. Il doit
        être retiré après usage via _cleanup_virtual_nodes().

        Returns:
            node_id (int, potentiellement virtuel/négatif) ou None si hors
            de portée.
        """
        best = None
        lat_rad = math.radians(lat)
        cos_lat = max(math.cos(lat_rad), 1e-6)
        xp, yp = lon * cos_lat, lat

        for u, v, data in self.graph.edges(data=True):
            lat_u, lon_u = self.graph.nodes[u]["lat"], self.graph.nodes[u]["lon"]
            lat_v, lon_v = self.graph.nodes[v]["lat"], self.graph.nodes[v]["lon"]
            xu, yu = lon_u * cos_lat, lat_u
            xv, yv = lon_v * cos_lat, lat_v
            dx, dy = xv - xu, yv - yu
            lensq = dx * dx + dy * dy
            t = 0.0 if lensq == 0 else max(0.0, min(1.0, ((xp - xu) * dx + (yp - yu) * dy) / lensq))
            xc, yc = xu + t * dx, yu + t * dy
            lat_c, lon_c = yc, xc / cos_lat
            d = haversine_distance(lat, lon, lat_c, lon_c)
            if best is None or d < best[0]:
                best = (d, u, v, t, data, lat_c, lon_c)

        if best is None or best[0] > max_radius_m:
            return None

        d, u, v, t, data, lat_c, lon_c = best
        if t <= 1e-6:
            return u
        if t >= 1 - 1e-6:
            return v

        # Le point projeté tombe au milieu de l'arête : on la scinde
        self._virtual_node_counter -= 1
        vid = self._virtual_node_counter
        self.graph.add_node(vid, lat=lat_c, lon=lon_c, virtual=True)
        d_u = haversine_distance(lat_c, lon_c, self.graph.nodes[u]["lat"], self.graph.nodes[u]["lon"])
        d_v = haversine_distance(lat_c, lon_c, self.graph.nodes[v]["lat"], self.graph.nodes[v]["lon"])
        self.graph.add_edge(vid, u, weight=d_u, distance_m=d_u,
                             highway=data.get("highway"), way_tags=data.get("way_tags"))
        self.graph.add_edge(vid, v, weight=d_v, distance_m=d_v,
                             highway=data.get("highway"), way_tags=data.get("way_tags"))
        self.graph.remove_edge(u, v)
        self._active_virtual_nodes.append((vid, u, v, data))
        return vid

    def _cleanup_virtual_nodes(self) -> None:
        """
        Retire les nœuds virtuels temporaires et restaure les arêtes scindées.

        Traité en ordre inverse de création (LIFO) : si un second point
        s'est accroché sur une arête créée par le premier découpage (les
        deux points sont proches), défaire dans le désordre recréerait un
        nœud virtuel fantôme sans coordonnées (via add_edge qui recrée
        silencieusement un nœud manquant).
        """
        for vid, u, v, data in reversed(self._active_virtual_nodes):
            if self.graph.has_node(vid):
                self.graph.remove_node(vid)
            if not self.graph.has_edge(u, v):
                self.graph.add_edge(u, v, **data)
        self._active_virtual_nodes = []

    # ------------------------------------------------------------------
    # Distance à la route la plus proche
    # ------------------------------------------------------------------

    def distance_to_nearest_road(self, lat: float, lon: float) -> float:
        """
        Calcule la distance minimale en mètres entre un point GPS et le réseau routier
        (en considérant les segments de route/arêtes, pas seulement les nœuds).
        """
        min_dist = float("inf")
        if not self.graph.edges:
            return min_dist

        # Facteur d'échelle pour la longitude selon la latitude moyenne
        lat_rad = math.radians(lat)
        cos_lat = math.cos(lat_rad)

        xp = lon * cos_lat
        yp = lat

        for u, v in self.graph.edges:
            lat_u = self.graph.nodes[u].get("lat")
            lon_u = self.graph.nodes[u].get("lon")
            lat_v = self.graph.nodes[v].get("lat")
            lon_v = self.graph.nodes[v].get("lon")

            if None in (lat_u, lon_u, lat_v, lon_v):
                continue

            xu, yu = lon_u * cos_lat, lat_u
            xv, yv = lon_v * cos_lat, lat_v

            dx = xv - xu
            dy = yv - yu
            lensq = dx*dx + dy*dy

            if lensq == 0:
                # Segment de longueur nulle (les deux nœuds coïncident)
                d = haversine_distance(lat, lon, lat_u, lon_u)
            else:
                # Projection de P sur le segment [U, V]
                t = ((xp - xu) * dx + (yp - yu) * dy) / lensq
                t = max(0.0, min(1.0, t))
                
                # Coordonnées du point le plus proche
                xc = xu + t * dx
                yc = yu + t * dy
                
                lat_c = yc
                lon_c = xc / cos_lat if cos_lat != 0 else xc
                
                d = haversine_distance(lat, lon, lat_c, lon_c)

            if d < min_dist:
                min_dist = d

        return min_dist

    def nearest_point_on_road(
        self, lat: float, lon: float
    ) -> tuple[float, float, float]:
        """
        Trouve le point exact le plus proche sur le réseau routier (projeté
        sur l'arête la plus proche, pas seulement le nœud le plus proche).

        Utilisé pour accrocher un point cliqué par l'utilisateur exactement
        sur la route quand il en est très proche, plutôt que de garder les
        coordonnées brutes du clic (qui peuvent être à quelques mètres à
        côté visuellement).

        Returns:
            (lat_proche, lon_proche, distance_m). Si le graphe n'a aucune
            arête, retourne (lat, lon, +inf) (aucun accrochage possible).
        """
        best_lat, best_lon = lat, lon
        min_dist = float("inf")
        if not self.graph.edges:
            return best_lat, best_lon, min_dist

        lat_rad = math.radians(lat)
        cos_lat = math.cos(lat_rad)
        xp, yp = lon * cos_lat, lat

        for u, v in self.graph.edges:
            lat_u = self.graph.nodes[u].get("lat")
            lon_u = self.graph.nodes[u].get("lon")
            lat_v = self.graph.nodes[v].get("lat")
            lon_v = self.graph.nodes[v].get("lon")

            if None in (lat_u, lon_u, lat_v, lon_v):
                continue

            xu, yu = lon_u * cos_lat, lat_u
            xv, yv = lon_v * cos_lat, lat_v

            dx = xv - xu
            dy = yv - yu
            lensq = dx * dx + dy * dy

            if lensq == 0:
                lat_c, lon_c = lat_u, lon_u
            else:
                t = ((xp - xu) * dx + (yp - yu) * dy) / lensq
                t = max(0.0, min(1.0, t))
                xc = xu + t * dx
                yc = yu + t * dy
                lat_c = yc
                lon_c = xc / cos_lat if cos_lat != 0 else xc

            d = haversine_distance(lat, lon, lat_c, lon_c)
            if d < min_dist:
                min_dist = d
                best_lat, best_lon = lat_c, lon_c

        return best_lat, best_lon, min_dist

    # ------------------------------------------------------------------
    # Calcul de l'itinéraire principal
    # ------------------------------------------------------------------

    def compute_route(
        self,
        start: Waypoint,
        end:   Waypoint,
        algorithm: str = "astar",
        nb_alternatives: int = 2,
    ) -> RouteResult:
        """
        Calcule l'itinéraire le plus court entre deux points GPS.

        Processus :
          1. Valider que les points sont dans la zone OSM.
          2. Trouver les nœuds du graphe les plus proches (snap).
          3. Vérifier que les nœuds sont dans la même composante connexe.
          4. Calculer le chemin principal (A* ou Dijkstra).
          5. Calculer des chemins alternatifs (suppression progressive d'arêtes).
          6. Retourner un RouteResult avec la séquence de waypoints.

        Args:
            start          : Waypoint de départ (lat, lon).
            end            : Waypoint d'arrivée (lat, lon).
            algorithm      : 'astar' (défaut) ou 'dijkstra'.
            nb_alternatives: Nombre de chemins alternatifs à calculer.

        Returns:
            RouteResult contenant les waypoints et les alternatives.

        Raises:
            ValueError : Si les points sont hors zone, hors réseau routier,
                         ou si aucun chemin n'existe.
        """
        # --- Validation géographique ---
        for pt, name in [(start, "départ"), (end, "arrivée")]:
            if not self.is_within_bounds(pt.lat, pt.lon):
                raise ValueError(
                    f"Le point de {name} (lat={pt.lat:.5f}, lon={pt.lon:.5f}) "
                    f"est en dehors de la zone couverte par le fichier OSM."
                )

        # --- Snap vers le nœud le plus proche ---
        start_node = self.nearest_node(start.lat, start.lon)
        end_node   = self.nearest_node(end.lat,   end.lon)

        if start_node is None:
            raise ValueError(
                f"Aucune route navigable trouvée à moins de {DEFAULT_SNAP_RADIUS_M} m "
                f"du point de départ. Vérifiez que vous avez cliqué sur une voie."
            )
        if end_node is None:
            raise ValueError(
                f"Aucune route navigable trouvée à moins de {DEFAULT_SNAP_RADIUS_M} m "
                f"du point d'arrivée. Vérifiez que vous avez cliqué sur une voie."
            )

        w_start = Waypoint(lat=start.lat, lon=start.lon, node_id=None)
        w_end = Waypoint(lat=end.lat, lon=end.lon, node_id=None)

        # Points très proches : même nœud routier ET géométriquement proches
        # → trajectoire directe + tour alternatif. Si les deux points
        # partagent seulement le même nœud "le plus proche" sans être
        # réellement proches l'un de l'autre (zone peu couverte par le
        # réseau routier), ce n'est PAS un cas de points confondus : on
        # laisse le calcul normal ci-dessous chercher un vrai chemin routier
        # plutôt que de supposer à tort qu'il n'y a rien à parcourir.
        if start_node == end_node and haversine_distance(
            start.lat, start.lon, end.lat, end.lon
        ) <= CONFOUNDED_POINTS_MAX_M:
            return self._compute_same_node_route(
                start, end, start_node, w_start, w_end, nb_alternatives
            )

        # --- Vérification de la connexité ---
        if not nx.has_path(self.graph, start_node, end_node):
            raise ValueError(
                "Impossible de calculer un itinéraire : les deux points "
                "ne sont pas connectés dans le réseau routier OSM."
            )

        # --- Accrochage précis sur l'arête la plus proche pour le calcul du
        # chemin (au lieu du simple nœud le plus proche) : évite qu'un nœud
        # mal placé par rapport à la direction du trajet ne force un
        # aller-retour inutile avant de repartir vers la destination.
        try:
            path_start_node = self._snap_point_to_graph(start.lat, start.lon) or start_node
            path_end_node = self._snap_point_to_graph(end.lat, end.lon) or end_node

            # --- Calcul du chemin principal ---
            print(f"[Route] Calcul {algorithm.upper()} de {path_start_node} → {path_end_node} ...")
            main_path = self._find_path(self.graph, path_start_node, path_end_node, algorithm)
            main_waypoints = self._path_to_waypoints(main_path)
            main_waypoints = self._merge_exact_endpoints(main_waypoints, w_start, w_end)

            # 2. Appliquer la condition d'interpolation de distance max = 10m
            main_waypoints = interpolate_waypoints(main_waypoints, max_dist=10.0)

            # 3. Calculer la distance aller-retour (doublée)
            main_distance = calculate_path_distance(main_waypoints) * 2.0

            # 4. Assigner la double numérotation (aller et retour) pour les waypoints
            self._assign_aller_retour_indices(main_waypoints)
            N_main = len(main_waypoints)

            print(f"[Route] Chemin principal interpolé : {N_main} waypoints, distance aller-retour : {main_distance:.1f} m")

            # --- Calcul des chemins alternatifs (Yen's k-shortest approximation) ---
            alternatives = self._compute_alternatives(
                path_start_node, path_end_node, main_path, nb_alternatives
            )

            # Appliquer la même logique d'interpolation et d'aller-retour aux alternatives
            final_alternatives = []
            for alt_wps in alternatives:
                alt_raw = self._merge_exact_endpoints(list(alt_wps), w_start, w_end)
                alt_interpolated = interpolate_waypoints(alt_raw, max_dist=10.0)
                self._assign_aller_retour_indices(alt_interpolated)
                final_alternatives.append(alt_interpolated)
        finally:
            self._cleanup_virtual_nodes()

        # Le trajet principal ET les alternatives suivent exclusivement le
        # graphe routier complet (A*/Dijkstra + suppression d'arêtes pour
        # les alternatives). Le "corridor" (raccourci le long des routes
        # proches de la ligne directe) n'est plus utilisé du tout, même en
        # alternative : il peut couper au plus court près d'une zone dense
        # (parking, petites voies résidentielles rapprochées) d'une façon
        # qui ne correspond pas au trajet attendu.

        # Trier les alternatives par distance croissante : l'itinéraire principal
        # reste garanti le plus court, les alternatives sont classées de la plus
        # courte à la plus longue.
        final_alternatives.sort(key=lambda wps: calculate_path_distance(wps) * 2.0)
        final_alternatives = final_alternatives[:nb_alternatives]
        for i, alt in enumerate(final_alternatives):
            print(f"[Route] Alternative {i + 1} (aller-retour) : "
                  f"{len(alt)} waypoints, {calculate_path_distance(alt) * 2.0:.1f} m")

        # Mise à jour des start/end avec les node_ids snappés
        start.node_id = start_node
        end.node_id   = end_node

        return RouteResult(
            waypoints=main_waypoints,
            total_distance_m=main_distance,
            path_node_ids=main_path,
            alternatives=final_alternatives,
        )

    def _leg_with_alternatives(
        self,
        leg_start: Waypoint,
        leg_end: Waypoint,
        algorithm: str,
        nb_alternatives: int,
    ) -> tuple[list[Waypoint], list[list[Waypoint]]]:
        """
        Calcule les waypoints (aller) d'un segment, avec ses alternatives
        éventuelles. Gère le cas des points réellement confondus au sein du
        trajet (pas de détour, pas d'alternative) — mais uniquement s'ils
        sont géométriquement proches, pas seulement parce qu'ils partagent
        le même nœud routier "le plus proche" (ce qui peut arriver entre
        deux points éloignés si le réseau routier est peu couvert autour
        d'eux, et donnerait sinon une ligne droite hors-route).
        """
        leg_start_node = self.nearest_node(leg_start.lat, leg_start.lon)
        leg_end_node = self.nearest_node(leg_end.lat, leg_end.lon)
        if (
            leg_start_node is not None
            and leg_start_node == leg_end_node
            and haversine_distance(
                leg_start.lat, leg_start.lon, leg_end.lat, leg_end.lon
            ) <= CONFOUNDED_POINTS_MAX_M
        ):
            wps = [
                Waypoint(lat=leg_start.lat, lon=leg_start.lon, node_id=None),
                Waypoint(lat=leg_end.lat, lon=leg_end.lon, node_id=None),
            ]
            return wps, []

        leg_result = self.compute_route(
            leg_start, leg_end, algorithm=algorithm, nb_alternatives=nb_alternatives
        )
        return leg_result.waypoints, leg_result.alternatives

    def _order_via_points_by_road(
        self,
        start: Waypoint,
        via_points: list[Waypoint],
        end: Waypoint,
        algorithm: str,
    ) -> list[Waypoint]:
        """
        Réordonne les points intermédiaires pour éviter les détours inutiles
        dus au simple ordre de clic (l'utilisateur choisit QUELS points
        imposer, pas dans quel ordre les visiter).

        Contrairement à une distance à vol d'oiseau, la distance ROUTIÈRE
        réelle est utilisée : sur un réseau avec détours obligés (bâtiments,
        boucles...), le point "le plus proche" en ligne droite peut en
        réalité nécessiter un grand détour, ce qui donnerait un ordre de
        visite illogique (revenir près du départ avant d'attaquer le reste
        du trajet). Heuristique : plus proche voisin, puis 2-opt, sur la
        matrice des distances routières entre chaque paire de points.
        """
        if len(via_points) < 2:
            return list(via_points)

        points = [start] + list(via_points) + [end]
        n = len(points)

        dist = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                wps, _ = self._leg_with_alternatives(points[i], points[j], algorithm, 0)
                d = calculate_path_distance(wps)
                dist[i][j] = d
                dist[j][i] = d

        remaining = list(range(1, n - 1))
        seq = [0]
        current = 0
        while remaining:
            nxt = min(remaining, key=lambda j: dist[current][j])
            seq.append(nxt)
            remaining.remove(nxt)
            current = nxt
        seq.append(n - 1)

        improved = True
        while improved:
            improved = False
            for i in range(1, len(seq) - 2):
                for j in range(i + 1, len(seq) - 1):
                    a, b = seq[i - 1], seq[i]
                    c, d = seq[j], seq[j + 1]
                    before = dist[a][b] + dist[c][d]
                    after = dist[a][c] + dist[b][d]
                    if after < before - 1e-6:
                        seq[i:j + 1] = list(reversed(seq[i:j + 1]))
                        improved = True

        return [points[idx] for idx in seq[1:-1]]

    def compute_route_via(
        self,
        waypoints: list[Waypoint],
        algorithm: str = "astar",
        nb_alternatives: int = 2,
    ) -> RouteResult:
        """
        Calcule un itinéraire aller-retour passant par une liste de points
        GPS (départ, points intermédiaires imposés par l'utilisateur,
        arrivée). L'ORDRE des points intermédiaires n'a pas besoin d'être
        celui du clic : ils sont automatiquement réordonnés selon la
        distance ROUTIÈRE réelle (_order_via_points_by_road), pas la
        distance à vol d'oiseau — évite de forcer un aller-retour vers un
        point géométriquement proche mais routièrement éloigné.

        Chaque segment consécutif est calculé indépendamment avec la même
        logique de sécurité que compute_route (chemin routier, corridor le
        long des routes proches, jamais de ligne droite hors-route), puis
        les segments sont mis bout à bout. Le trajet retour emprunte le
        même chemin en sens inverse.

        Des itinéraires alternatifs sont proposés en substituant, un segment
        à la fois, un chemin alternatif de ce segment (corridor différent,
        autre chemin routier...) dans le trajet complet.

        Args:
            waypoints      : Liste d'au moins 2 Waypoint : le premier est le
                             départ, le dernier l'arrivée (fixes), les points
                             entre les deux sont réordonnés automatiquement.
            algorithm      : 'astar' (défaut) ou 'dijkstra'.
            nb_alternatives: Nombre d'itinéraires alternatifs à proposer.

        Returns:
            RouteResult avec le trajet principal et ses alternatives.

        Raises:
            ValueError : Si moins de 2 points sont fournis.
        """
        if len(waypoints) < 2:
            raise ValueError(
                "Il faut au moins un point de départ et un point d'arrivée."
            )

        start, end = waypoints[0], waypoints[-1]
        via_points = self._order_via_points_by_road(
            start, waypoints[1:-1], end, algorithm
        )
        ordered_points = [start] + via_points + [end]

        leg_main_wps: list[list[Waypoint]] = []
        leg_alts: list[list[list[Waypoint]]] = []
        for i in range(len(ordered_points) - 1):
            wps, alts = self._leg_with_alternatives(
                ordered_points[i], ordered_points[i + 1], algorithm, nb_alternatives
            )
            leg_main_wps.append(wps)
            leg_alts.append(alts)

        def _concat_legs(legs: list[list[Waypoint]]) -> list[Waypoint]:
            result: list[Waypoint] = []
            for leg in legs:
                if result and leg:
                    leg = leg[1:]
                result.extend(leg)
            return result

        full_waypoints = _concat_legs(leg_main_wps)
        self._assign_aller_retour_indices(full_waypoints)
        total_distance = calculate_path_distance(full_waypoints) * 2.0

        # --- Itinéraires alternatifs : un segment substitué à la fois ---
        final_alternatives: list[list[Waypoint]] = []
        for i, alts in enumerate(leg_alts):
            for alt_leg_wps in alts:
                if len(final_alternatives) >= nb_alternatives:
                    break
                substituted = list(leg_main_wps)
                substituted[i] = alt_leg_wps
                alt_full = _concat_legs(substituted)
                self._assign_aller_retour_indices(alt_full)
                final_alternatives.append(alt_full)
            if len(final_alternatives) >= nb_alternatives:
                break

        print(f"[Route] Itinéraire multi-points ({len(ordered_points)} points imposés) : "
              f"{len(full_waypoints)} waypoints, distance aller-retour : "
              f"{total_distance:.1f} m, {len(final_alternatives)} alternative(s)")

        return RouteResult(
            waypoints=full_waypoints,
            total_distance_m=total_distance,
            path_node_ids=[],
            alternatives=final_alternatives,
        )

    # ------------------------------------------------------------------
    # Méthodes internes de pathfinding
    # ------------------------------------------------------------------

    def _compute_same_node_route(
        self,
        start: Waypoint,
        end: Waypoint,
        node: int,
        w_start: Waypoint,
        w_end: Waypoint,
        nb_alternatives: int,
    ) -> RouteResult:
        """
        Gère le cas où départ et arrivée sont confondus (même nœud routier).

        Le robot sort vers le nœud voisin le plus proche puis revient au
        point de départ/arrivée. Trajectoire aller-retour complète (les deux
        trajets sont mis bout à bout dans la liste), waypoints numérotés
        séquentiellement 1..2N (pas de double numérotation aller/retour).
        """
        print(
            f"[Route] Points confondus (même nœud {node}) — "
            "sortie vers le nœud le plus proche puis retour."
        )

        tours = self._find_nearest_neighbor_tours(node, 1 + nb_alternatives)

        def _build_tour(node_path: list[int]) -> list[Waypoint]:
            out_wps = self._path_to_waypoints(node_path)
            back_wps = self._path_to_waypoints(list(reversed(node_path)))
            full = out_wps + back_wps[1:]
            full = self._merge_exact_endpoints(full, w_start, w_end)
            wps = interpolate_waypoints(full, max_dist=10.0)
            self._assign_single_indices(wps)
            return wps

        if tours:
            main_path = tours[0]
            main_waypoints = _build_tour(main_path)
            final_alternatives = [
                _build_tour(p) for p in tours[1:]
            ]
        else:
            main_path = [node]
            main_waypoints = interpolate_waypoints(
                [Waypoint(lat=w_start.lat, lon=w_start.lon, node_id=None)],
                max_dist=10.0,
            )
            self._assign_single_indices(main_waypoints)
            final_alternatives = []

        main_distance = calculate_path_distance(main_waypoints)

        print(
            f"[Route] Chemin principal (aller-retour) : {len(main_waypoints)} waypoints, "
            f"distance : {main_distance:.1f} m"
        )
        for i, alt in enumerate(final_alternatives):
            print(
                f"[Route] Alternative {i + 1} (aller-retour) : {len(alt)} waypoints, "
                f"{calculate_path_distance(alt):.1f} m"
            )

        start.node_id = node
        end.node_id = node

        return RouteResult(
            waypoints=main_waypoints,
            total_distance_m=main_distance,
            path_node_ids=main_path,
            alternatives=final_alternatives,
            one_way=True,
        )

    def _merge_exact_endpoints(
        self,
        waypoints: list[Waypoint],
        w_start: Waypoint,
        w_end: Waypoint,
    ) -> list[Waypoint]:
        """Injecte les coordonnées exactes de départ et d'arrivée."""
        merged = list(waypoints)
        if merged:
            if haversine_distance(w_start.lat, w_start.lon, merged[0].lat, merged[0].lon) > 0.1:
                merged.insert(0, Waypoint(lat=w_start.lat, lon=w_start.lon, node_id=None))
            else:
                merged[0].lat = w_start.lat
                merged[0].lon = w_start.lon

            if haversine_distance(w_end.lat, w_end.lon, merged[-1].lat, merged[-1].lon) > 0.1:
                merged.append(Waypoint(lat=w_end.lat, lon=w_end.lon, node_id=None))
            else:
                merged[-1].lat = w_end.lat
                merged[-1].lon = w_end.lon
        else:
            merged = [
                Waypoint(lat=w_start.lat, lon=w_start.lon, node_id=None),
                Waypoint(lat=w_end.lat, lon=w_end.lon, node_id=None),
            ]
        return merged

    def _assign_aller_retour_indices(self, waypoints: list[Waypoint]) -> None:
        """Assigne les index aller/retour à une liste de waypoints, puis
        repère les points physiquement revisités plus de deux fois."""
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            wp.index_aller = i + 1
            wp.index_retour = 2 * n - 1 - i
            wp.passages = None
        self._detect_repeated_passages(waypoints)

    def _detect_repeated_passages(
        self, waypoints: list[Waypoint], tolerance_m: float = 1.5
    ) -> None:
        """
        Regroupe les waypoints correspondant au même point physique (ex:
        une boucle qui recroise son propre tracé) et, pour tout point
        traversé plus de deux fois (aller + retour confondus), liste tous
        ses numéros de passage dans wp.passages. Les points normaux
        (traversés une seule fois dans chaque sens) ne sont pas modifiés.
        """
        n = len(waypoints)
        visited = [False] * n
        for i in range(n):
            if visited[i]:
                continue
            group = [i]
            for j in range(i + 1, n):
                if visited[j]:
                    continue
                if haversine_distance(
                    waypoints[i].lat, waypoints[i].lon,
                    waypoints[j].lat, waypoints[j].lon,
                ) <= tolerance_m:
                    group.append(j)
            for idx in group:
                visited[idx] = True
            if len(group) < 2:
                continue

            passages = []
            for idx in group:
                wp = waypoints[idx]
                if wp.index_aller is not None:
                    passages.append(wp.index_aller)
                if wp.index_retour is not None:
                    passages.append(wp.index_retour)
            if len(passages) > 2:
                passages = sorted(set(passages))
                for idx in group:
                    waypoints[idx].passages = passages

    def _assign_single_indices(self, waypoints: list[Waypoint]) -> None:
        """Assigne une numérotation simple 1..N (aller seul, sans retour)."""
        for i, wp in enumerate(waypoints):
            wp.index_aller = i + 1
            wp.index_retour = None

    def _find_nearest_neighbor_tours(self, start_node: int, nb: int) -> list[list[int]]:
        """
        Cherche des trajets aller-retour vers les voisins directs les plus
        proches de start_node (le nœud le plus proche suffit à constituer
        le tour le plus court ; les voisins suivants servent d'alternatives).
        """
        neighbors = list(self.graph.neighbors(start_node))
        neighbors.sort(
            key=lambda n: self.graph.get_edge_data(start_node, n).get("weight", float("inf"))
        )
        return [[start_node, n] for n in neighbors[:nb]]

    def _find_path(self, graph: nx.Graph, source: int, target: int,
                   algorithm: str) -> list[int]:
        """
        Calcule un chemin dans le graphe donné.

        Args:
            graph     : Graphe NetworkX à utiliser.
            source    : Nœud de départ.
            target    : Nœud d'arrivée.
            algorithm : 'astar' ou 'dijkstra'.

        Returns:
            Liste ordonnée d'identifiants de nœuds.
        """
        if algorithm == "astar":
            # Heuristique A* : distance haversine entre le nœud courant et la cible
            def heuristic(n, goal):
                lat_n = graph.nodes[n].get("lat", 0)
                lon_n = graph.nodes[n].get("lon", 0)
                lat_g = graph.nodes[goal].get("lat", 0)
                lon_g = graph.nodes[goal].get("lon", 0)
                return haversine_distance(lat_n, lon_n, lat_g, lon_g)

            return nx.astar_path(
                graph, source, target,
                heuristic=heuristic,
                weight="weight",
            )
        else:
            # Dijkstra classique — garanti optimal mais plus lent sur grands graphes
            return nx.dijkstra_path(
                graph, source, target, weight="weight"
            )

    def _path_to_waypoints(
        self, path: list[int], graph: Optional[nx.Graph] = None
    ) -> list[Waypoint]:
        """
        Convertit une liste d'IDs de nœuds en liste de Waypoints GPS.

        Args:
            path  : Liste d'identifiants de nœuds OSM.
            graph : Graphe source (par défaut self.graph).

        Returns:
            Liste de Waypoint(lat, lon, node_id).
        """
        g = graph if graph is not None else self.graph
        waypoints = []
        for nid in path:
            lat = g.nodes[nid].get("lat")
            lon = g.nodes[nid].get("lon")
            if lat is not None and lon is not None:
                waypoints.append(Waypoint(lat=lat, lon=lon, node_id=nid))
        return waypoints

    def _path_distance(self, graph: nx.Graph, path: list[int]) -> float:
        """
        Calcule la distance totale d'un chemin en mètres.

        Args:
            graph : Graphe contenant les arêtes pondérées.
            path  : Liste ordonnée d'IDs de nœuds.

        Returns:
            Distance totale en mètres.
        """
        total = 0.0
        for i in range(len(path) - 1):
            edge_data = graph.get_edge_data(path[i], path[i + 1]) or {}
            total += edge_data.get("weight", 0.0)
        return total

    def _compute_alternatives(
        self,
        start_node: int,
        end_node:   int,
        main_path:  list[int],
        nb:         int,
    ) -> list[list[Waypoint]]:
        """
        Génère des chemins alternatifs en supprimant progressivement des arêtes
        du chemin principal (méthode inspirée de Yen's k-shortest paths, simplifiée).

        Stratégie :
          - On retire une arête du milieu du chemin principal.
          - On recalcule un chemin. S'il est différent du principal, on le garde.
          - On répète pour d'autres arêtes afin d'obtenir `nb` alternatives.

        Args:
            start_node : Nœud de départ.
            end_node   : Nœud d'arrivée.
            main_path  : Chemin principal (liste d'IDs).
            nb         : Nombre d'alternatives souhaitées.

        Returns:
            Liste de listes de Waypoints (une liste par alternative).
        """
        alternatives = []
        seen_paths   = {tuple(main_path)}

        # On tente de supprimer des arêtes à différentes positions du chemin
        step = max(1, len(main_path) // (nb + 1))
        positions = [step * i for i in range(1, nb + 2)]

        for pos in positions:
            if len(alternatives) >= nb:
                break
            if pos >= len(main_path) - 1:
                continue

            n1 = main_path[pos]
            n2 = main_path[pos + 1]

            # Vérifier que l'arête existe avant de la retirer
            if not self.graph.has_edge(n1, n2):
                continue

            # Sauvegarder et retirer l'arête temporairement
            edge_data = self.graph.get_edge_data(n1, n2).copy()
            self.graph.remove_edge(n1, n2)

            try:
                if nx.has_path(self.graph, start_node, end_node):
                    alt_path = self._find_path(
                        self.graph, start_node, end_node, "dijkstra"
                    )
                    path_key = tuple(alt_path)
                    if path_key not in seen_paths:
                        seen_paths.add(path_key)
                        alt_wps = self._path_to_waypoints(alt_path)
                        alternatives.append(alt_wps)
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                pass
            finally:
                # Toujours remettre l'arête
                self.graph.add_edge(n1, n2, **edge_data)

        return alternatives

    # ------------------------------------------------------------------
    # Export JSON
    # ------------------------------------------------------------------

    def export_json(
        self,
        result: RouteResult,
        output_file: str,
        origin: Optional[Waypoint] = None,
        default_speed: int = 3,
    ) -> str:
        """
        Exporte le résultat de l'itinéraire au format "Record" (points +
        segments avec coordonnées locales x/y, orientations de dôme caméra,
        vitesses, pauses...), compatible avec le format utilisé par les
        autres outils PGuard.

        Points non déduits de nos données (valeurs par défaut choisies) :
          - "critic_point"   : True pour le départ et l'arrivée, False ailleurs.
          - "dome_orientations", "light", "detection", "switch_cam",
            "turn_speed"     : valeurs neutres par défaut (True / [[0,0,3]]).
          - "pause"/"pause_duration" : False/0 (pas de modèle de pause ici).
          - "max_speed"/"speed"      : constante `default_speed` pour tous
                                        les segments (pas de profil de
                                        vitesse calculé).
          - "avoiding"       : False (pas de détection d'obstacles ici).
          - "safe_point_id"  : -2, "start_threshold" : -1 (constantes fixes).
          - "orientation" (segment) : changement de cap réel entre segments
                                       consécutifs, quantifié au multiple de
                                       90° le plus proche (0/±90/180).

        Args:
            result        : Résultat retourné par compute_route() /
                             compute_route_via().
            output_file   : Chemin du fichier JSON à créer.
            origin        : Point de référence pour les coordonnées locales
                             x/y (mètres). Par défaut : le premier waypoint
                             du trajet (x=0, y=0 au départ).
            default_speed : Vitesse constante appliquée à tous les segments
                             (pas de profil de vitesse calculé par le moteur).

        Returns:
            Chemin absolu du fichier créé.
        """
        waypoints = result.waypoints
        if not waypoints:
            raise ValueError("Aucun waypoint à exporter.")

        origin_wp = origin if origin is not None else waypoints[0]
        to_xy = _local_xy_projector(origin_wp.lat, origin_wp.lon)

        bearings = [
            bearing_deg(waypoints[i].lat, waypoints[i].lon, waypoints[i + 1].lat, waypoints[i + 1].lon)
            for i in range(len(waypoints) - 1)
        ]

        points_json = []
        for i, wp in enumerate(waypoints):
            x, y = to_xy(wp.lat, wp.lon)
            ways = [j for j in (i - 1, i) if 0 <= j < len(waypoints) - 1]
            points_json.append({
                "critic_point": i == 0 or i == len(waypoints) - 1,
                "detection": True,
                "dome_orientations": [[0, 0, 3]],
                "id": i,
                "light": True,
                "location": {
                    "latitude": wp.lat,
                    "longitude": wp.lon,
                    "x": x,
                    "y": y,
                },
                "pause": False,
                "pause_duration": 0,
                "switch_cam": True,
                "turn_speed": True,
                "ways": ways,
            })

        segments_json = []
        for i in range(len(waypoints) - 1):
            delta = bearings[i] - bearings[i - 1] if i > 0 else 0.0
            segments_json.append({
                "avoiding": False,
                "detection": True,
                "id": i,
                "max_speed": default_speed,
                "orientation": _quantize_turn_deg(delta),
                "points": [i, i + 1],
                "speed": default_speed,
            })

        data = {
            "distance": round(result.total_distance_m),
            "id": "Record",
            "one_way": result.one_way,
            "origin": {"latitude": origin_wp.lat, "longitude": origin_wp.lon},
            "points": points_json,
            "safe_point_id": -2,
            "segments": segments_json,
            "start_threshold": -1,
        }

        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        print(f"[Export] Trajectoire exportée (format Record) → '{output_file}' "
              f"({len(points_json)} points, {len(segments_json)} segments)")
        return output_file


# ---------------------------------------------------------------------------
# Point d'entrée en mode console (test rapide sans interface graphique)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="PGuard Route Engine — Calcul d'itinéraire depuis la console"
    )
    parser.add_argument("osm_file",          help="Chemin vers le fichier .osm")
    parser.add_argument("start_lat",  type=float, help="Latitude du départ")
    parser.add_argument("start_lon",  type=float, help="Longitude du départ")
    parser.add_argument("end_lat",    type=float, help="Latitude de l'arrivée")
    parser.add_argument("end_lon",    type=float, help="Longitude de l'arrivée")
    parser.add_argument("--output",   default="trajectory.json",
                        help="Fichier de sortie JSON (défaut: trajectory.json)")
    parser.add_argument("--algo",     default="astar",
                        choices=["astar", "dijkstra"],
                        help="Algorithme de pathfinding (défaut: astar)")
    args = parser.parse_args()

    engine = PGuardRouteEngine(args.osm_file)

    start_wp = Waypoint(lat=args.start_lat, lon=args.start_lon)
    end_wp   = Waypoint(lat=args.end_lat,   lon=args.end_lon)

    try:
        result = engine.compute_route(start_wp, end_wp, algorithm=args.algo)
        engine.export_json(result, args.output)
        print(f"\n✓ Trajectoire calculée avec succès !")
        print(f"  Distance totale : {result.total_distance_m:.1f} m")
        print(f"  Nombre de waypoints : {len(result.waypoints)}")
        print(f"  Alternatives disponibles : {len(result.alternatives)}")
    except ValueError as e:
        print(f"\n✗ Erreur : {e}")
