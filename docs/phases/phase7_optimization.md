# Phase 7: Optimization & Tuning

After basic integration works, optimize for your environment and mission profile.

---

## SLAM Parameters

### Indoor / Small Spaces
```yaml
surroundingKeyframeSearchRadius: 25.0       # (default: 50) - reduce search radius
surroundingkeyframeAddingDistThreshold: 0.5  # (default: 1.0) - more frequent keyframes
odometrySurfLeafSize: 0.1                    # (default: 0.2) - higher resolution
mappingCornerLeafSize: 0.05                  # (default: 0.1)
```

### Outdoor / Large Spaces
```yaml
surroundingKeyframeSearchRadius: 100.0       # increase search radius
surroundingkeyframeAddingDistThreshold: 2.0  # less frequent keyframes
odometrySurfLeafSize: 0.4                    # lower resolution for speed
mappingCornerLeafSize: 0.2
```

---

## ArduPilot Flight Tuning

### Aggressive Flight
```
WPNAV_SPEED,300      # 3.0 m/s
WPNAV_ACCEL,500      # 5.0 m/s²
PSC_POSXY_P,2.0      # Higher P gain
```

### Smooth / Precision Flight
```
WPNAV_SPEED,150      # 1.5 m/s
WPNAV_ACCEL,200      # 2.0 m/s²
PSC_POSXY_P,1.0      # Lower P gain
```

---

## Resource Optimization

### High CPU Usage
```yaml
# SLAM config
downsampleRate: 2              # Skip every other scan
mappingProcessInterval: 0.2    # Slower mapping rate

# Reduce feature extraction
edgeFeatureMinValidNum: 10     # (default: 5) - filter weak edges
surfFeatureMinValidNum: 200    # (default: 100) - filter weak surfaces
```

### High RAM Usage
```yaml
# Reduce map size in memory
globalMapVisualizationSearchRadius: 50.0  # (default: 1000)
surroundingKeyframeSize: 25               # (default: 50)
```

---

## Output
Document final tuning parameters in your config files. Return to Phase 5 for re-testing with optimized settings.
