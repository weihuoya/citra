package org.citra.emu;

import android.database.Cursor;
import android.database.MatrixCursor;
import android.os.CancellationSignal;
import android.os.ParcelFileDescriptor;
import android.provider.DocumentsContract;
import android.provider.DocumentsProvider;
import android.webkit.MimeTypeMap;

import androidx.annotation.Nullable;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Collections;
import java.util.LinkedList;

public final class UserPathProvider extends DocumentsProvider {

    private final String[] DEFAULT_ROOT_PROJECTION = new String[] {
        DocumentsContract.Root.COLUMN_ROOT_ID,
        DocumentsContract.Root.COLUMN_MIME_TYPES,
        DocumentsContract.Root.COLUMN_FLAGS,
        DocumentsContract.Root.COLUMN_ICON,
        DocumentsContract.Root.COLUMN_TITLE,
        DocumentsContract.Root.COLUMN_SUMMARY,
        DocumentsContract.Root.COLUMN_DOCUMENT_ID,
        DocumentsContract.Root.COLUMN_AVAILABLE_BYTES
    };

    private final String[] DEFAULT_DOCUMENT_PROJECTION = new String[] {
        DocumentsContract.Document.COLUMN_DOCUMENT_ID,
        DocumentsContract.Document.COLUMN_MIME_TYPE,
        DocumentsContract.Document.COLUMN_DISPLAY_NAME,
        DocumentsContract.Document.COLUMN_LAST_MODIFIED,
        DocumentsContract.Document.COLUMN_FLAGS,
        DocumentsContract.Document.COLUMN_SIZE
    };

    private String ROOT_ID;
    private File BASE_DIR;

    private String getRelativePath(File file) {
        if (file == BASE_DIR) {
            return "";
        }
        return file.getAbsolutePath().substring(BASE_DIR.getAbsolutePath().length() + 1);
    }

    /**
     * Get the document id given a file (The reverse of @{link #getDocumentId}).
     */
    private File getFile(String documentId) throws FileNotFoundException {
        if (documentId.startsWith(ROOT_ID)) {
            File file = new File(BASE_DIR, documentId.substring(ROOT_ID.length() + 1));
            if (!file.exists()) {
                throw new FileNotFoundException("document not found");
            }
            return file;
        } else {
            throw new FileNotFoundException("document is not in any known root");
        }
    }

    /**
     * Get the file given a document id (the reverse of {@link #getFile}).
     */
    private String getDocumentId(File file) {
        String path = getRelativePath(file);
        return ROOT_ID + "/" + path;
    }

    @Override
    public Cursor queryRoots(String[] projection) throws FileNotFoundException {
        String title = getContext().getString(R.string.app_name);
        int flags = DocumentsContract.Root.FLAG_SUPPORTS_CREATE;
        flags |= DocumentsContract.Root.FLAG_SUPPORTS_SEARCH;
        flags |= DocumentsContract.Root.FLAG_SUPPORTS_IS_CHILD;

        MatrixCursor cursor = new MatrixCursor(projection != null ? projection : DEFAULT_ROOT_PROJECTION);
        MatrixCursor.RowBuilder row = cursor.newRow();
        row.add(DocumentsContract.Root.COLUMN_ROOT_ID, ROOT_ID);
        row.add(DocumentsContract.Root.COLUMN_FLAGS, flags);
        row.add(DocumentsContract.Root.COLUMN_TITLE, title);
        row.add(DocumentsContract.Root.COLUMN_DOCUMENT_ID, getDocumentId(BASE_DIR));
        row.add(DocumentsContract.Root.COLUMN_MIME_TYPES, "*/*");
        row.add(DocumentsContract.Root.COLUMN_AVAILABLE_BYTES, BASE_DIR.getFreeSpace());
        row.add(DocumentsContract.Root.COLUMN_ICON, R.mipmap.ic_citra);
        return cursor;
    }

    @Override
    public Cursor queryDocument(String documentId, String[] projection) throws FileNotFoundException {
        MatrixCursor cursor = new MatrixCursor(projection != null ? projection : DEFAULT_DOCUMENT_PROJECTION);
        includeFile(cursor, documentId, null);
        return cursor;
    }

    @Override
    public Cursor queryChildDocuments(String parentDocumentId, String[] projection, String sortOrder) throws FileNotFoundException {
        MatrixCursor cursor = new MatrixCursor(projection != null ? projection : DEFAULT_DOCUMENT_PROJECTION);
        File parent = getFile(parentDocumentId);
        for (File file : parent.listFiles()) {
            includeFile(cursor, null, file);
        }
        return cursor;
    }

    @Override
    public ParcelFileDescriptor openDocument(String documentId, String mode, @Nullable CancellationSignal signal) throws FileNotFoundException {
        File file = getFile(documentId);
        int accessMode = ParcelFileDescriptor.parseMode(mode);
        return ParcelFileDescriptor.open(file, accessMode);
    }

    @Override
    public void deleteDocument(String documentId) throws FileNotFoundException {
        File file = getFile(documentId);
        if (!file.delete()) {
            throw new FileNotFoundException("Failed to delete document with id " + documentId);
        }
    }

    @Override
    public String getDocumentType(String documentId) throws FileNotFoundException {
        File file = getFile(documentId);
        return getMimeType(file);
    }

    @Override
    public Cursor querySearchDocuments(String rootId, String query, String[] projection) throws FileNotFoundException {
        MatrixCursor cursor = new MatrixCursor(projection != null ? projection : DEFAULT_DOCUMENT_PROJECTION);
        File parent = getFile(rootId);

        // This example implementation searches file names for the query and doesn't rank search
        // results, so we can stop as soon as we find a sufficient number of matches.  Other
        // implementations might rank results and use other data about files, rather than the file
        // name, to produce a match.
        LinkedList<File> pending = new LinkedList<>();
        pending.add(parent);

        int MAX_SEARCH_RESULTS = 50;
        while (!pending.isEmpty() && cursor.getCount() < MAX_SEARCH_RESULTS) {
            final File file = pending.removeFirst();
            if (file.isDirectory()) {
                Collections.addAll(pending, file.listFiles());
            } else {
                if (file.getName().toLowerCase().contains(query)) {
                    includeFile(cursor, null, file);
                }
            }
        }

        return cursor;
    }

    @Override
    public boolean onCreate() {
        ROOT_ID = "root";
        BASE_DIR = getContext().getExternalFilesDir(null);
        return true;
    }

    /**
     * Add a representation of a file to a cursor.
     *
     * @param cursor the cursor to modify
     * @param documentId  the document ID representing the desired file (may be null if given file)
     * @param file   the File object representing the desired file (may be null if given docID)
     */
    private void includeFile(MatrixCursor cursor, String documentId, File file) throws FileNotFoundException {
        if (documentId == null) {
            documentId = getDocumentId(file);
        } else if (file == null) {
            file = getFile(documentId);
        }

        String mimeType = getMimeType(file);

        int flags = 0;
        if (file.isDirectory()) {
            if (file.canWrite()) {
                flags = DocumentsContract.Document.FLAG_DIR_SUPPORTS_CREATE;
            }
        } else if (file.canWrite()) {
            flags = DocumentsContract.Document.FLAG_SUPPORTS_WRITE;
            flags = flags | DocumentsContract.Document.FLAG_SUPPORTS_DELETE;
            flags = flags | DocumentsContract.Document.FLAG_SUPPORTS_REMOVE;
            flags = flags | DocumentsContract.Document.FLAG_SUPPORTS_MOVE;
            flags = flags | DocumentsContract.Document.FLAG_SUPPORTS_COPY;
            flags = flags | DocumentsContract.Document.FLAG_SUPPORTS_RENAME;
            if (mimeType.startsWith("image/")) {
                flags |= DocumentsContract.Document.FLAG_SUPPORTS_THUMBNAIL;
            }
        }

        MatrixCursor.RowBuilder row = cursor.newRow();
        row.add(DocumentsContract.Document.COLUMN_DOCUMENT_ID, documentId);
        row.add(DocumentsContract.Document.COLUMN_SIZE, file.length());
        row.add(DocumentsContract.Document.COLUMN_MIME_TYPE, mimeType);
        row.add(DocumentsContract.Document.COLUMN_LAST_MODIFIED, file.lastModified());
        row.add(DocumentsContract.Document.COLUMN_FLAGS, flags);
        row.add(DocumentsContract.Root.COLUMN_ICON, R.mipmap.ic_citra);

        if (file == BASE_DIR) {
            String title = getContext().getString(R.string.app_name);
            row.add(DocumentsContract.Document.COLUMN_DISPLAY_NAME, title);
        } else {
            row.add(DocumentsContract.Document.COLUMN_DISPLAY_NAME, file.getName());
        }
    }

    private static String getMimeType(File file) {
        if (file.isDirectory()) {
            return DocumentsContract.Document.MIME_TYPE_DIR;
        } else {
            final String name = file.getName();
            final int lastDot = name.lastIndexOf('.');
            if (lastDot >= 0) {
                final String extension = name.substring(lastDot + 1).toLowerCase();
                final String mime = MimeTypeMap.getSingleton().getMimeTypeFromExtension(extension);
                if (mime != null) return mime;
            }
            return "application/octet-stream";
        }
    }
}