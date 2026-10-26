using UnityEngine;

[RequireComponent(typeof(Terrain))]
public class TerrainGenerator : MonoBehaviour
{
    public float maxHillHeight = 4f;      // meters; 0 gives flat ground (Levels 1 and 2)
    public float hillSize = 50f;          // meters per hill; bigger = gentler slopes
    public bool randomizeHeight = true;   // pick a height between 0 and max every episode

    private Terrain terrain;
    private TerrainCollider terrainCollider;
    private float[,] heights;

    void Awake()
    {
        terrain = GetComponent<Terrain>();
        terrainCollider = GetComponent<TerrainCollider>();

        // Work on a copy, so playing never changes the terrain asset in the project
        TerrainData copy = Instantiate(terrain.terrainData);
        terrain.terrainData = copy;
        if (terrainCollider != null) terrainCollider.terrainData = copy;
    }

    public void Generate()
    {
        TerrainData data = terrain.terrainData;
        int res = data.heightmapResolution;
        if (heights == null || heights.GetLength(0) != res) heights = new float[res, res];

        float hillHeight = randomizeHeight ? Random.Range(0f, maxHillHeight) : maxHillHeight;
        float amplitude = Mathf.Clamp01(hillHeight / data.size.y); // heights are stored as 0..1 of the terrain's max height

        float offsetX = Random.Range(0f, 10000f); // a new random pattern every time
        float offsetZ = Random.Range(0f, 10000f);
        float scaleX = data.size.x / hillSize / (res - 1);
        float scaleZ = data.size.z / hillSize / (res - 1);

        for (int z = 0; z < res; z++)
        {
            for (int x = 0; x < res; x++)
            {
                heights[z, x] = Mathf.PerlinNoise(offsetX + x * scaleX, offsetZ + z * scaleZ) * amplitude;
            }
        }

        data.SetHeights(0, 0, heights);
        Physics.SyncTransforms();
    }
}