import React, { useState } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function PersonalizePage() {
  const { siteConfig } = useDocusaurusContext();
  const [userBackground, setUserBackground] = useState({
    experience: '',
    researchInterest: '',
    learningGoal: '',
    preferredModules: []
  });
  const [isSaved, setIsSaved] = useState(false);

  const handleInputChange = (field, value) => {
    setUserBackground(prev => ({
      ...prev,
      [field]: value
    }));
  };

  const handleModuleToggle = (module) => {
    setUserBackground(prev => {
      const newModules = prev.preferredModules.includes(module)
        ? prev.preferredModules.filter(m => m !== module)
        : [...prev.preferredModules, module];

      return {
        ...prev,
        preferredModules: newModules
      };
    });
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    // In a real implementation, this would save to backend
    console.log('Saving user background:', userBackground);
    setIsSaved(true);
    setTimeout(() => setIsSaved(false), 3000);
  };

  const modules = ['ROS2', 'Gazebo/Unity', 'Isaac', 'VLA', 'Capstone'];

  return (
    <Layout
      title={`Personalize Learning - ${siteConfig.title}`}
      description="Personalize your Physical AI and Humanoid Robotics learning experience">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset--2">
            <h1>Personalize Your Learning Experience</h1>
            <p>Tell us about your background and interests to customize your textbook experience.</p>

            {isSaved && (
              <div className="alert alert--success margin-bottom--md" role="alert">
                Your preferences have been saved! The content will be personalized based on your background.
              </div>
            )}

            <form onSubmit={handleSubmit}>
              <div className="margin-bottom--lg">
                <label htmlFor="experience" className="form-label">
                  <strong>Experience Level</strong>
                </label>
                <select
                  id="experience"
                  className="form-select"
                  value={userBackground.experience}
                  onChange={(e) => handleInputChange('experience', e.target.value)}
                >
                  <option value="">Select your experience level</option>
                  <option value="beginner">Beginner (Little to no experience)</option>
                  <option value="intermediate">Intermediate (Some robotics/AI background)</option>
                  <option value="advanced">Advanced (Significant robotics/AI experience)</option>
                  <option value="researcher">Researcher (Academic or industry researcher)</option>
                </select>
              </div>

              <div className="margin-bottom--lg">
                <label htmlFor="researchInterest" className="form-label">
                  <strong>Research Interests</strong>
                </label>
                <input
                  type="text"
                  id="researchInterest"
                  className="form-control"
                  placeholder="e.g., Manipulation, Navigation, Humanoid Control..."
                  value={userBackground.researchInterest}
                  onChange={(e) => handleInputChange('researchInterest', e.target.value)}
                />
              </div>

              <div className="margin-bottom--lg">
                <label className="form-label">
                  <strong>Preferred Modules</strong>
                </label>
                <p>Which modules are you most interested in?</p>
                {modules.map((module) => (
                  <label className="checkbox margin-right--lg" key={module}>
                    <input
                      type="checkbox"
                      checked={userBackground.preferredModules.includes(module)}
                      onChange={() => handleModuleToggle(module)}
                    />
                    <span>{module}</span>
                  </label>
                ))}
              </div>

              <div className="margin-bottom--lg">
                <label htmlFor="learningGoal" className="form-label">
                  <strong>Learning Goals</strong>
                </label>
                <textarea
                  id="learningGoal"
                  className="form-control"
                  rows="3"
                  placeholder="What do you hope to achieve by studying this textbook?"
                  value={userBackground.learningGoal}
                  onChange={(e) => handleInputChange('learningGoal', e.target.value)}
                />
              </div>

              <button type="submit" className="button button--primary button--lg">
                Save Preferences
              </button>
            </form>

            <div className="margin-vert--lg">
              <h2>How Personalization Works</h2>
              <ul>
                <li>Content difficulty adjusts based on your experience level</li>
                <li>Recommended modules based on your interests</li>
                <li>Customized examples and exercises</li>
                <li>Progress tracking tailored to your goals</li>
                <li>Relevant research papers and resources suggested</li>
              </ul>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default PersonalizePage;