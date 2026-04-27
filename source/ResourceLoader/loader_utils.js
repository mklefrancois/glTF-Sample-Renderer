/**
 * Utility class providing static helper methods for resource loading operations,
 * such as extracting file extensions, resolving folder paths, normalizing relative
 * paths, and detecting absolute URLs.
 */
class ResourceLoaderUtils {
    /**
     * Extracts the file extension from a filename.
     * @param {string} filename - The filename or path to extract the extension from.
     * @returns {string|undefined} The lowercase file extension (without the leading dot),
     *   or `undefined` if the filename has no extension.
     */
    static getExtension(filename) {
        const split = filename.toLowerCase().split(".");
        if (split.length == 1) {
            return undefined;
        }
        return split[split.length - 1];
    }

    /**
     * Returns the directory portion of a file path, including the trailing slash.
     * @param {string} filePath - The full file path.
     * @returns {string} The path up to and including the last `/`, or an empty string
     *   if no `/` is present.
     */
    static getContainingFolder(filePath) {
        return filePath.substring(0, filePath.lastIndexOf("/") + 1);
    }

    /**
     * Normalizes a relative URL path by resolving `.` and `..` segments.
     * - Strips a leading `./` prefix.
     * - Collapses `/./` sequences to `/`.
     * - Resolves `/../` sequences by removing the preceding path segment.
     * @param {string} relativePath - The relative path to clean.
     * @returns {string} The normalized path with dot segments resolved.
     */
    static cleanRelativePath(relativePath) {
        if (relativePath.startsWith("./")) {
            relativePath = relativePath.substring(2);
        }
        while (relativePath.includes("/./")) {
            relativePath = relativePath.replace("/./", "/");
        }
        let searchIndex = relativePath.indexOf("/../");
        while (searchIndex !== -1) {
            let slashIndex = relativePath.lastIndexOf("/", searchIndex - 1);
            relativePath =
                relativePath.substring(0, slashIndex + 1) + relativePath.substring(searchIndex + 4);
            searchIndex = relativePath.indexOf("/../");
        }
        return relativePath;
    }

    /**
     * Determines whether a URL is absolute (i.e. contains a scheme such as `http:` or `data:`).
     * A URL is considered absolute when it contains a `:` that appears before any `/`.
     * @param {string} url - The URL string to test.
     * @returns {boolean} `true` if the URL is absolute, `false` otherwise.
     */
    static isAbsoluteUrl(url) {
        const colonIndex = url.indexOf(":");
        const slashIndex = url.indexOf("/");
        return colonIndex !== -1 && (slashIndex === -1 || colonIndex < slashIndex);
    }
}

export { ResourceLoaderUtils };
