package org.gravity.ros.analysis.messages;

import java.util.LinkedList;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IResourceVisitor;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.jface.text.IDocument;
import org.eclipse.ui.editors.text.TextFileDocumentProvider;
import org.eclipse.ui.texteditor.IDocumentProvider;
import org.python.pydev.core.IPythonNature;
import org.python.pydev.core.IPythonPathNature;
import org.python.pydev.core.MisconfigurationException;
import org.python.pydev.parser.PyParser;
import org.python.pydev.parser.PyParser.ParserInfo;
import org.python.pydev.parser.jython.SimpleNode;
import org.python.pydev.plugin.nature.PythonNature;
import org.python.pydev.shared_core.model.ISimpleNode;
import org.python.pydev.shared_core.parsing.BaseParser.ParseOutput;

public class PythonProjectParser {

	public void parse(IProject project) throws CoreException {
		IPythonPathNature iPythonPathNature = PythonNature.getPythonPathNature(project);
		IPythonNature iPythonNature = iPythonPathNature.getNature();
		
		LinkedList<ParseOutput> parsedList = new LinkedList<>();
		IDocumentProvider provider = new TextFileDocumentProvider();
		
		for (IResource scr: iPythonPathNature.getProjectSourcePathFolderSet()) {
			scr.accept(new IResourceVisitor() {
				@Override
				public boolean visit(IResource resource) throws CoreException {
					if (resource instanceof IFile)
					{
						IFile ifile = (IFile)resource;
						String ifileName = ifile.getName(); // TEST LINE
						
						// Ignoriere __init__.py files
						if (ifile.getFileExtension().equals("py") && !ifileName.equals("__init__.py"))
						{
							System.out.println(ifileName); // TEST LINE
							provider.connect(ifile);
							IDocument document = provider.getDocument(ifile);
							
							try {
								ParseOutput parseOutput = PyParser.createCythonAst(new ParserInfo(document, iPythonNature));
								parsedList.add(parseOutput);
							} catch (MisconfigurationException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}
					}
					return true;
				}
			});
		}
		/* NEW TEST FOR PARSE OUTPUT */
		ParseContentPrint(parsedList);
	}
	
	public void ParseContentPrint(LinkedList<ParseOutput> parsedList) {
		for (ParseOutput parseOutput: parsedList) {
			SimpleNode ast = (SimpleNode)parseOutput.ast;
			System.out.println(ast.toString());
		}
	}
}
