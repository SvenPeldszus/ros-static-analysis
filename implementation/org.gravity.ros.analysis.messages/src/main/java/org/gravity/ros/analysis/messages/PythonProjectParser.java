package org.gravity.ros.analysis.messages;

import org.eclipse.core.resources.IProject;
import org.python.pydev.parser.PyParser;

public class PythonProjectParser {

	public void parse(IProject project) {
		//TODO: Implement something, probably using the following API:
		PyParser.createCythonAst(null);
	}
}
